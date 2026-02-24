/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <drogon/drogon.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../web-ifc/parsing/IfcLoader.h"
#include "../web-ifc/schema/IfcSchemaManager.h"

namespace
{
#ifndef WEB_IFC_API_DEFAULT_PORT
#define WEB_IFC_API_DEFAULT_PORT 18080
#endif

    static_assert(WEB_IFC_API_DEFAULT_PORT > 0 && WEB_IFC_API_DEFAULT_PORT <= 65535, "WEB_IFC_API_DEFAULT_PORT must be in range 1..65535");

    constexpr uint32_t kDefaultTapeSize = 64U * 1024U * 1024U;
    constexpr uint64_t kDefaultMemoryLimit = 2ULL * 1024ULL * 1024ULL * 1024ULL;
    constexpr uint32_t kDefaultLineWriterBuffer = 10'000U;
    constexpr uint32_t kDefaultTopTypes = 20U;
    constexpr uint16_t kDefaultPort = static_cast<uint16_t>(WEB_IFC_API_DEFAULT_PORT);

    Json::Value BuildErrorPayload(const std::string &message)
    {
        Json::Value payload;
        payload["ok"] = false;
        payload["error"] = message;
        return payload;
    }

    std::string ResolveFilePathFromRequest(const drogon::HttpRequestPtr &req)
    {
        auto filePath = req->getParameter("filePath");
        if (filePath.empty())
        {
            filePath = req->getParameter("path");
        }

        if (!filePath.empty())
        {
            return filePath;
        }

        auto body = req->getJsonObject();
        if (body != nullptr)
        {
            if ((*body).isMember("filePath") && (*body)["filePath"].isString())
            {
                return (*body)["filePath"].asString();
            }
            if ((*body).isMember("path") && (*body)["path"].isString())
            {
                return (*body)["path"].asString();
            }
        }

        return {};
    }

    uint32_t ResolveTopNFromRequest(const drogon::HttpRequestPtr &req)
    {
        auto text = req->getParameter("top");
        if (text.empty())
        {
            return kDefaultTopTypes;
        }

        try
        {
            auto raw = std::stoul(text);
            if (raw == 0)
            {
                return kDefaultTopTypes;
            }

            return static_cast<uint32_t>(std::min<unsigned long>(raw, 200UL));
        }
        catch (...)
        {
            return kDefaultTopTypes;
        }
    }

    uint16_t ResolvePort(int argc, char **argv)
    {
        auto parsePort = [](const std::string &value) -> std::optional<uint16_t>
        {
            try
            {
                auto parsed = std::stoul(value);
                if (parsed > 0 && parsed <= 65535U)
                {
                    return static_cast<uint16_t>(parsed);
                }
            }
            catch (...)
            {
            }
            return std::nullopt;
        };

        if (argc > 1 && argv != nullptr && argv[1] != nullptr)
        {
            auto fromArg = parsePort(argv[1]);
            if (fromArg.has_value())
            {
                return *fromArg;
            }
        }

        if (const char *envPort = std::getenv("WEB_IFC_API_PORT"); envPort != nullptr)
        {
            auto fromEnv = parsePort(envPort);
            if (fromEnv.has_value())
            {
                return *fromEnv;
            }
        }

        return kDefaultPort;
    }

    Json::Value BuildIfcSummary(const std::string &filePath, uint32_t topN)
    {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open())
        {
            throw std::runtime_error("Could not open IFC file: " + filePath);
        }

        webifc::schema::IfcSchemaManager schemaManager;
        webifc::parsing::IfcLoader loader(
            kDefaultTapeSize,
            kDefaultMemoryLimit,
            kDefaultLineWriterBuffer,
            schemaManager);

        auto parseStart = std::chrono::steady_clock::now();
        loader.LoadFile(file);
        auto parseEnd = std::chrono::steady_clock::now();
        auto parseMs = std::chrono::duration_cast<std::chrono::milliseconds>(parseEnd - parseStart).count();

        auto allLines = loader.GetAllLines();
        std::unordered_map<uint32_t, uint64_t> typeCounts;
        typeCounts.reserve(allLines.size());

        for (auto expressId : allLines)
        {
            auto typeCode = loader.GetLineType(expressId);
            if (typeCode == 0)
            {
                continue;
            }
            typeCounts[typeCode]++;
        }

        std::vector<std::pair<uint32_t, uint64_t>> sorted(typeCounts.begin(), typeCounts.end());
        std::sort(
            sorted.begin(),
            sorted.end(),
            [&](const auto &left, const auto &right)
            {
                if (left.second != right.second)
                {
                    return left.second > right.second;
                }
                return schemaManager.IfcTypeCodeToType(left.first) < schemaManager.IfcTypeCodeToType(right.first);
            });

        Json::Value response;
        response["ok"] = true;
        response["filePath"] = filePath;
        response["schema"] = std::string(schemaManager.GetSchemaName(loader.GetSchema()));
        response["parseTimeMs"] = Json::Int64(parseMs);
        response["lineCount"] = Json::UInt64(allLines.size());
        response["maxExpressId"] = loader.GetMaxExpressId();
        response["uniqueTypeCount"] = Json::UInt64(typeCounts.size());

        Json::Value topTypes(Json::arrayValue);
        auto limit = std::min<size_t>(topN, sorted.size());
        for (size_t i = 0; i < limit; i++)
        {
            Json::Value item;
            item["typeCode"] = sorted[i].first;
            item["typeName"] = schemaManager.IfcTypeCodeToType(sorted[i].first);
            item["count"] = Json::UInt64(sorted[i].second);
            topTypes.append(item);
        }
        response["topTypes"] = std::move(topTypes);

        return response;
    }
} // namespace

int main(int argc, char **argv)
{
    using namespace drogon;

    app().registerHandler(
        "/health",
        [](const HttpRequestPtr &, std::function<void(const HttpResponsePtr &)> &&callback)
        {
            Json::Value payload;
            payload["ok"] = true;
            payload["service"] = "web-ifc-drogon-api";
            payload["status"] = "healthy";

            auto response = HttpResponse::newHttpJsonResponse(payload);
            response->setStatusCode(k200OK);
            callback(response);
        },
        {Get});

    app().registerHandler(
        "/api/ifc/summary",
        [](const HttpRequestPtr &req, std::function<void(const HttpResponsePtr &)> &&callback)
        {
            auto fail = [&](HttpStatusCode code, const std::string &message)
            {
                auto response = HttpResponse::newHttpJsonResponse(BuildErrorPayload(message));
                response->setStatusCode(code);
                callback(response);
            };

            auto filePath = ResolveFilePathFromRequest(req);
            if (filePath.empty())
            {
                fail(k400BadRequest, "Missing filePath. Use query string (?filePath=...) or JSON body {\"filePath\":\"...\"}.");
                return;
            }

            auto topN = ResolveTopNFromRequest(req);

            try
            {
                auto payload = BuildIfcSummary(filePath, topN);
                auto response = HttpResponse::newHttpJsonResponse(payload);
                response->setStatusCode(k200OK);
                callback(response);
            }
            catch (const std::exception &ex)
            {
                fail(k500InternalServerError, ex.what());
            }
            catch (...)
            {
                fail(k500InternalServerError, "Unexpected server error while parsing IFC file.");
            }
        },
        {Get, Post});

    auto port = ResolvePort(argc, argv);
    app().setLogLevel(trantor::Logger::kInfo);
    app().addListener("0.0.0.0", port);
    app().setThreadNum(std::max(1U, std::thread::hardware_concurrency()));

    LOG_INFO << "Starting web-ifc Drogon API on http://0.0.0.0:" << port;
    app().run();
    return 0;
}
