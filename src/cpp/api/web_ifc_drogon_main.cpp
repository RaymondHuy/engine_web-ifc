/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <drogon/drogon.h>
#include <json/json.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "web-ifc/parsing/IfcLoader.h"
#include "web-ifc/modelmanager/ModelManager.h"
#include "web-ifc/schema/IfcSchemaManager.h"

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
    constexpr size_t kDefaultMaxRequestBodyBytes = 512ULL * 1024ULL * 1024ULL;
    constexpr size_t kDefaultMaxMemoryBodyBytes = 2ULL * 1024ULL * 1024ULL;
    constexpr const char *kDefaultUploadDir = "/tmp/web-ifc-uploads";

    Json::Value BuildErrorPayload(const std::string &message)
    {
        Json::Value payload;
        payload["ok"] = false;
        payload["error"] = message;
        return payload;
    }

    void AddCorsHeaders(const drogon::HttpRequestPtr &req, const drogon::HttpResponsePtr &resp)
    {
        const auto &origin = req->getHeader("Origin");
        resp->addHeader("Access-Control-Allow-Origin", origin.empty() ? "*" : origin);
        resp->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");

        const auto &requestHeaders = req->getHeader("Access-Control-Request-Headers");
        resp->addHeader(
            "Access-Control-Allow-Headers",
            requestHeaders.empty() ? "Content-Type, Accept" : requestHeaders);
        resp->addHeader("Access-Control-Max-Age", "86400");
        resp->addHeader("Vary", "Origin");
    }

    void SetupCors()
    {
        drogon::app().registerSyncAdvice([](const drogon::HttpRequestPtr &req) -> drogon::HttpResponsePtr
                                          {
            if (req->method() != drogon::HttpMethod::Options)
            {
                return {};
            }

            auto response = drogon::HttpResponse::newHttpResponse();
            response->setStatusCode(drogon::k204NoContent);
            AddCorsHeaders(req, response);
            return response; });

        drogon::app().registerPostHandlingAdvice(
            [](const drogon::HttpRequestPtr &req, const drogon::HttpResponsePtr &resp)
            {
                AddCorsHeaders(req, resp);
            });
    }

    std::string ToLowerAscii(std::string value)
    {
        std::transform(
            value.begin(),
            value.end(),
            value.begin(),
            [](const unsigned char ch)
            { return static_cast<char>(std::tolower(ch)); });
        return value;
    }

    std::string TrimAscii(std::string value)
    {
        const auto notSpace = [](const unsigned char ch)
        { return !std::isspace(ch); };
        value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
        value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
        return value;
    }

    std::optional<std::string> ExtractBoundaryFromContentType(const std::string &contentType)
    {
        auto lower = ToLowerAscii(contentType);
        auto boundaryPos = lower.find("boundary=");
        if (boundaryPos == std::string::npos)
        {
            return std::nullopt;
        }

        auto valuePos = boundaryPos + std::strlen("boundary=");
        if (valuePos >= contentType.size())
        {
            return std::nullopt;
        }

        std::string boundary;
        if (contentType[valuePos] == '"')
        {
            const auto endQuote = contentType.find('"', valuePos + 1);
            if (endQuote == std::string::npos)
            {
                return std::nullopt;
            }
            boundary = contentType.substr(valuePos + 1, endQuote - (valuePos + 1));
        }
        else
        {
            const auto endPos = contentType.find(';', valuePos);
            boundary = contentType.substr(valuePos, endPos == std::string::npos ? std::string::npos : endPos - valuePos);
            boundary = TrimAscii(boundary);
        }

        if (boundary.empty())
        {
            return std::nullopt;
        }

        if (boundary.rfind("--", 0) == 0)
        {
            boundary.erase(0, 2);
        }

        if (boundary.empty())
        {
            return std::nullopt;
        }

        return boundary;
    }

    std::string ExtractFilenameFromDisposition(const std::string &contentDisposition)
    {
        auto lower = ToLowerAscii(contentDisposition);
        auto filenamePos = lower.find("filename=");
        if (filenamePos == std::string::npos)
        {
            return {};
        }

        auto valuePos = filenamePos + std::strlen("filename=");
        if (valuePos >= contentDisposition.size())
        {
            return {};
        }

        if (contentDisposition[valuePos] == '"')
        {
            const auto endQuote = contentDisposition.find('"', valuePos + 1);
            if (endQuote == std::string::npos)
            {
                return {};
            }
            return contentDisposition.substr(valuePos + 1, endQuote - (valuePos + 1));
        }

        const auto endPos = contentDisposition.find(';', valuePos);
        auto value = contentDisposition.substr(valuePos, endPos == std::string::npos ? std::string::npos : endPos - valuePos);
        return TrimAscii(value);
    }

    bool ParseFirstMultipartFile(
        const std::string &contentType,
        const std::string &body,
        std::string &outFileName,
        std::string &outFileBytes,
        std::string &outError)
    {
        auto boundaryOpt = ExtractBoundaryFromContentType(contentType);
        if (!boundaryOpt.has_value())
        {
            outError = "Invalid Content-Type: missing multipart boundary.";
            return false;
        }

        const auto boundary = "--" + *boundaryOpt;
        std::size_t searchPos = 0;

        while (true)
        {
            const auto partBegin = body.find(boundary, searchPos);
            if (partBegin == std::string::npos)
            {
                outError = "No multipart parts found.";
                return false;
            }

            auto afterBoundary = partBegin + boundary.size();
            if (afterBoundary + 1 < body.size() && body[afterBoundary] == '-' && body[afterBoundary + 1] == '-')
            {
                outError = "Multipart ended without file part.";
                return false;
            }

            if (afterBoundary + 1 < body.size() && body[afterBoundary] == '\r' && body[afterBoundary + 1] == '\n')
            {
                afterBoundary += 2;
            }
            else if (afterBoundary < body.size() && body[afterBoundary] == '\n')
            {
                afterBoundary += 1;
            }

            const auto headersEnd = body.find("\r\n\r\n", afterBoundary);
            if (headersEnd == std::string::npos)
            {
                outError = "Malformed multipart body: missing header separator.";
                return false;
            }

            const auto headersBlock = body.substr(afterBoundary, headersEnd - afterBoundary);
            const auto dataStart = headersEnd + 4;
            const auto nextBoundary = body.find("\r\n" + boundary, dataStart);
            if (nextBoundary == std::string::npos)
            {
                outError = "Malformed multipart body: missing next boundary.";
                return false;
            }

            std::string disposition;
            std::size_t lineStart = 0;
            while (lineStart < headersBlock.size())
            {
                const auto lineEnd = headersBlock.find("\r\n", lineStart);
                const auto line = headersBlock.substr(
                    lineStart,
                    lineEnd == std::string::npos ? std::string::npos : lineEnd - lineStart);

                auto lineLower = ToLowerAscii(line);
                if (lineLower.rfind("content-disposition:", 0) == 0)
                {
                    disposition = line.substr(std::strlen("Content-Disposition:"));
                    disposition = TrimAscii(disposition);
                }

                if (lineEnd == std::string::npos)
                {
                    break;
                }
                lineStart = lineEnd + 2;
            }

            auto fileName = ExtractFilenameFromDisposition(disposition);
            if (!fileName.empty())
            {
                outFileName = fileName;
                outFileBytes.assign(body.data() + dataStart, nextBoundary - dataStart);
                return true;
            }

            searchPos = nextBoundary + 2;
        }
    }

    std::string ResolveUploadFileNameFromRequest(const drogon::HttpRequestPtr &req)
    {
        auto fileName = req->getParameter("fileName");
        if (fileName.empty())
        {
            fileName = req->getParameter("filename");
        }
        if (fileName.empty())
        {
            fileName = req->getHeader("x-file-name");
        }
        if (fileName.empty())
        {
            fileName = req->getHeader("x-filename");
        }

        if (fileName.empty())
        {
            auto body = req->getJsonObject();
            if (body != nullptr)
            {
                if ((*body).isMember("fileName") && (*body)["fileName"].isString())
                {
                    fileName = (*body)["fileName"].asString();
                }
                else if ((*body).isMember("filename") && (*body)["filename"].isString())
                {
                    fileName = (*body)["filename"].asString();
                }
            }
        }

        return fileName;
    }

    template <typename TRequest, typename = void>
    struct HasGetBodyMethod : std::false_type
    {
    };

    template <typename TRequest>
    struct HasGetBodyMethod<TRequest, std::void_t<decltype(std::declval<const TRequest &>().getBody())>> : std::true_type
    {
    };

    template <typename TRequest, typename = void>
    struct HasBodyMethod : std::false_type
    {
    };

    template <typename TRequest>
    struct HasBodyMethod<TRequest, std::void_t<decltype(std::declval<const TRequest &>().body())>> : std::true_type
    {
    };

    template <typename TRequestPtr>
    std::string ReadRequestBody(const TRequestPtr &req)
    {
        if (!req)
        {
            return {};
        }

        using RequestType = std::remove_reference_t<decltype(*req)>;
        if constexpr (HasGetBodyMethod<RequestType>::value)
        {
            const auto body = req->getBody();
            return std::string(body.data(), body.size());
        }
        else if constexpr (HasBodyMethod<RequestType>::value)
        {
            const auto body = req->body();
            return std::string(body.data(), body.size());
        }
        else
        {
            return {};
        }
    }

    std::string SanitizeFileName(const std::string &input)
    {
        std::string out;
        out.reserve(input.size());

        for (const auto ch : input)
        {
            const bool allowed =
                (ch >= 'a' && ch <= 'z') ||
                (ch >= 'A' && ch <= 'Z') ||
                (ch >= '0' && ch <= '9') ||
                ch == '.' || ch == '_' || ch == '-';
            out.push_back(allowed ? ch : '_');
        }

        while (!out.empty() && (out.front() == '.' || out.front() == '/' || out.front() == '\\'))
        {
            out.erase(out.begin());
        }

        if (out.empty())
        {
            const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count();
            out = "upload-" + std::to_string(nowMs) + ".ifc";
        }

        if (out.find('.') == std::string::npos)
        {
            out += ".ifc";
        }

        return out;
    }

    std::filesystem::path ResolveUploadDirectory()
    {
        if (const char *envDir = std::getenv("WEB_IFC_UPLOAD_DIR"); envDir != nullptr && std::strlen(envDir) > 0)
        {
            return std::filesystem::path(envDir);
        }
        return std::filesystem::path(kDefaultUploadDir);
    }

    std::filesystem::path BuildUniqueUploadPath(const std::filesystem::path &dir, const std::string &sanitizedFileName)
    {
        auto candidate = dir / sanitizedFileName;
        if (!std::filesystem::exists(candidate))
        {
            return candidate;
        }

        const auto stem = candidate.stem().string();
        const auto ext = candidate.extension().string();
        for (uint32_t i = 1; i <= 10'000; i++)
        {
            auto withSuffix = dir / (stem + "-" + std::to_string(i) + ext);
            if (!std::filesystem::exists(withSuffix))
            {
                return withSuffix;
            }
        }

        const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
        return dir / (stem + "-" + std::to_string(nowMs) + ext);
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

    size_t ResolveMaxRequestBodyBytes()
    {
        const char *envValue = std::getenv("WEB_IFC_API_MAX_BODY_BYTES");
        if (envValue == nullptr || std::strlen(envValue) == 0)
        {
            return kDefaultMaxRequestBodyBytes;
        }

        try
        {
            return std::max<size_t>(std::stoull(envValue), 1024ULL * 1024ULL);
        }
        catch (...)
        {
            return kDefaultMaxRequestBodyBytes;
        }
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

    std::vector<char> ReadFileBytes(const std::string &filePath)
    {
        std::ifstream file(filePath, std::ios::binary | std::ios::ate);
        if (!file.is_open())
        {
            throw std::runtime_error("Could not open IFC file: " + filePath);
        }

        const auto fileSize = file.tellg();
        if (fileSize < 0)
        {
            throw std::runtime_error("Could not read IFC file size: " + filePath);
        }

        std::vector<char> bytes(static_cast<size_t>(fileSize));
        file.seekg(0, std::ios::beg);
        if (!bytes.empty() && !file.read(bytes.data(), static_cast<std::streamsize>(bytes.size())))
        {
            throw std::runtime_error("Could not read IFC file: " + filePath);
        }

        return bytes;
    }

    webifc::manager::LoaderSettings BuildDefaultGeometrySettings()
    {
        webifc::manager::LoaderSettings settings;
        settings.COORDINATE_TO_ORIGIN = true;
        settings.CIRCLE_SEGMENTS = 6;
        settings.TOLERANCE_PLANE_INTERSECTION = 1.0e-04;
        settings.TOLERANCE_PLANE_DEVIATION = 1.0e-04;
        settings.TOLERANCE_BACK_DEVIATION_DISTANCE = 1.0e-04;
        settings.TOLERANCE_INSIDE_OUTSIDE_PERIMETER = 1.0e-10;
        settings.TOLERANCE_SCALAR_EQUALITY = 1.0e-04;
        settings.PLANE_REFIT_ITERATIONS = 10;
        settings.BOOLEAN_UNION_THRESHOLD = 150;
        return settings;
    }

    Json::Value BuildNumberArray(const std::array<double, 16> &values)
    {
        Json::Value array(Json::arrayValue);
        for (const auto value : values)
        {
            array.append(value);
        }
        return array;
    }

    Json::Value BuildColorArray(const glm::dvec4 &color)
    {
        Json::Value array(Json::arrayValue);
        array.append(color.x);
        array.append(color.y);
        array.append(color.z);
        array.append(color.w);
        return array;
    }

    Json::Value BuildVertexArray(webifc::geometry::IfcGeometry &geometry)
    {
        geometry.GetVertexData();

        Json::Value array(Json::arrayValue);
        for (const auto value : geometry.fvertexData)
        {
            array.append(value);
        }
        return array;
    }

    Json::Value BuildIndexArray(const webifc::geometry::IfcGeometry &geometry)
    {
        Json::Value array(Json::arrayValue);
        for (const auto value : geometry.indexData)
        {
            array.append(Json::UInt(value));
        }
        return array;
    }

    Json::Value BuildIfcGeometry(const std::string &filePath)
    {
        auto bytes = ReadFileBytes(filePath);
        if (bytes.empty())
        {
            throw std::runtime_error("IFC file is empty: " + filePath);
        }

        webifc::manager::ModelManager manager(false);
        auto modelId = manager.CreateModel(BuildDefaultGeometrySettings());

        const std::function<uint32_t(char *, size_t, size_t)> loaderFunc =
            [&bytes](char *dest, size_t sourceOffset, size_t destSize) -> uint32_t
        {
            if (sourceOffset >= bytes.size())
            {
                return 0;
            }

            const auto remaining = bytes.size() - sourceOffset;
            const auto readSize = std::min(remaining, destSize);
            std::memcpy(dest, bytes.data() + sourceOffset, readSize);
            return static_cast<uint32_t>(readSize);
        };

        auto *loader = manager.GetIfcLoader(modelId);
        loader->LoadFile(loaderFunc);

        auto *geometryProcessor = manager.GetGeometryProcessor(modelId);
        const auto &schemaManager = manager.GetSchemaManager();

        Json::Value geometries(Json::arrayValue);
        Json::Value errors(Json::arrayValue);
        uint64_t meshCount = 0;
        uint64_t geometryCount = 0;
        uint64_t vertexCount = 0;
        uint64_t triangleCount = 0;

        for (const auto type : schemaManager.GetIfcElementList())
        {
            if (type == webifc::schema::IFCOPENINGELEMENT ||
                type == webifc::schema::IFCSPACE ||
                type == webifc::schema::IFCOPENINGSTANDARDCASE)
            {
                continue;
            }

            const auto elements = loader->GetExpressIDsWithType(type);
            const auto typeName = schemaManager.IfcTypeCodeToType(type);

            for (const auto expressId : elements)
            {
                try
                {
                    auto flatMesh = geometryProcessor->GetFlatMesh(expressId);
                    bool hasGeometry = false;

                    for (const auto &placed : flatMesh.geometries)
                    {
                        auto &geometry = geometryProcessor->GetGeometry(placed.geometryExpressID);
                        if (geometry.indexData.empty() || geometry.vertexData.empty())
                        {
                            continue;
                        }

                        Json::Value item;
                        item["expressID"] = Json::UInt(expressId);
                        item["type"] = typeName;
                        item["geometryExpressID"] = Json::UInt(placed.geometryExpressID);
                        item["color"] = BuildColorArray(placed.color);
                        item["transform"] = BuildNumberArray(placed.flatTransformation);
                        item["vertexData"] = BuildVertexArray(geometry);
                        item["indexData"] = BuildIndexArray(geometry);

                        vertexCount += geometry.fvertexData.size() / webifc::geometry::VERTEX_FORMAT_SIZE_FLOATS;
                        triangleCount += geometry.indexData.size() / 3U;
                        geometryCount++;
                        hasGeometry = true;
                        geometries.append(std::move(item));
                    }

                    if (hasGeometry)
                    {
                        meshCount++;
                    }
                    geometryProcessor->Clear();
                }
                catch (const std::exception &ex)
                {
                    if (errors.size() < 20)
                    {
                        Json::Value error;
                        error["expressID"] = Json::UInt(expressId);
                        error["type"] = typeName;
                        error["message"] = ex.what();
                        errors.append(std::move(error));
                    }
                    geometryProcessor->Clear();
                }
            }
        }

        Json::Value response;
        response["ok"] = true;
        response["filePath"] = filePath;
        response["schema"] = std::string(schemaManager.GetSchemaName(loader->GetSchema()));
        response["meshCount"] = Json::UInt64(meshCount);
        response["geometryCount"] = Json::UInt64(geometryCount);
        response["vertexCount"] = Json::UInt64(vertexCount);
        response["triangleCount"] = Json::UInt64(triangleCount);
        response["geometries"] = std::move(geometries);
        response["errors"] = std::move(errors);
        return response;
    }
} // namespace

int main(int argc, char **argv)
{
    using namespace drogon;

    SetupCors();

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
        "/api/ifc/upload",
        [](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& cb) {
            MultiPartParser parser;
            if (parser.parse(req) != 0) {
                auto resp = HttpResponse::newHttpJsonResponse(Json::Value{
                    {"error", "Invalid multipart/form-data or parse failed"}
                });
                resp->setStatusCode(k400BadRequest);
                return cb(resp);
            }

            // Optional: đọc các field text
            // auto params = parser.getParameters();
            // std::string userId = params["userId"];

            auto files = parser.getFiles();
            if (files.empty()) {
                auto resp = HttpResponse::newHttpJsonResponse(Json::Value{
                    {"error", "No file found. Expected field name: file or files"}
                });
                resp->setStatusCode(k400BadRequest);
                return cb(resp);
            }

            std::filesystem::create_directories("./uploads");

            Json::Value out;
            out["ok"] = true;
            out["count"] = (Json::UInt64)files.size();
            out["files"] = Json::arrayValue;

            for (auto& f : files) {
                // f: HttpFile
                // f.getItemName() : tên field trong form (vd "file" / "files")
                // f.getFileName() : tên file client gửi lên
                // f.getFileExtension() : ".png" ...
                // f.getFilePath() : đường dẫn file tạm drogon đã lưu

                // Tạo tên lưu an toàn
                auto safeName = utils::getUuid() + "_" + f.getFileName();
                std::string savePath = "./uploads/" + safeName;

                // Lưu file: move từ temp sang path
                // saveAs() sẽ copy/move từ file tạm drogon
                f.saveAs(savePath);

                Json::Value fi;
                fi["field"] = f.getItemName();
                fi["originalName"] = f.getFileName();
                fi["savedAs"] = safeName;
                fi["path"] = savePath;
                fi["sizeBytes"] = (Json::UInt64)std::filesystem::file_size(savePath);

                out["files"].append(fi);
            }

            cb(HttpResponse::newHttpJsonResponse(out));
        },
        {Post}
    );

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

    app().registerHandler(
        "/api/ifc/geometry",
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

            try
            {
                auto payload = BuildIfcGeometry(filePath);
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
                fail(k500InternalServerError, "Unexpected server error while building IFC geometry.");
            }
        },
        {Get, Post});

    auto port = ResolvePort(argc, argv);
    auto maxRequestBodyBytes = ResolveMaxRequestBodyBytes();
    app().setLogLevel(trantor::Logger::kInfo);
    app().setClientMaxBodySize(maxRequestBodyBytes);
    app().setClientMaxMemoryBodySize(std::min(kDefaultMaxMemoryBodyBytes, maxRequestBodyBytes));
    app().addListener("0.0.0.0", port);
    app().setThreadNum(std::max(1U, std::thread::hardware_concurrency()));

    LOG_INFO << "Starting web-ifc Drogon API on http://0.0.0.0:" << port
             << " with max request body " << maxRequestBodyBytes << " bytes";
    app().run();
    return 0;
}
