/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "../web-ifc/parsing/IfcLoader.h"
#include "../web-ifc/schema/IfcSchemaManager.h"
#include "../web-ifc/geometry/IfcGeometryLoader.h"
#include "../web-ifc/geometry/IfcGeometryProcessor.h"

namespace
{
    enum class Projection
    {
        XY,
        XZ,
        YZ
    };

    double Area2D(const std::vector<glm::dvec3> &ring, Projection projection)
    {
        if (ring.size() < 3)
        {
            return 0;
        }

        double area = 0;
        for (size_t i = 0; i < ring.size(); i++)
        {
            const auto &a = ring[i];
            const auto &b = ring[(i + 1) % ring.size()];
            switch (projection)
            {
            case Projection::XY:
                area += a.x * b.y - b.x * a.y;
                break;
            case Projection::XZ:
                area += a.x * b.z - b.x * a.z;
                break;
            default:
                area += a.y * b.z - b.y * a.z;
                break;
            }
        }

        return std::abs(area * 0.5);
    }

    Projection BestProjection(const std::vector<glm::dvec3> &ring)
    {
        const auto areaXY = Area2D(ring, Projection::XY);
        const auto areaXZ = Area2D(ring, Projection::XZ);
        const auto areaYZ = Area2D(ring, Projection::YZ);
        if (areaXY >= areaXZ && areaXY >= areaYZ)
        {
            return Projection::XY;
        }

        if (areaXZ >= areaYZ)
        {
            return Projection::XZ;
        }

        return Projection::YZ;
    }

    const char *ProjectionName(Projection projection)
    {
        switch (projection)
        {
        case Projection::XY:
            return "XY";
        case Projection::XZ:
            return "XZ";
        default:
            return "YZ";
        }
    }

    std::string ReadFile(const std::string &path)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
        {
            throw std::runtime_error("Could not open file: " + path);
        }

        file.seekg(0, std::ios::end);
        const auto size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::string buffer(static_cast<size_t>(size), '\0');
        if (!file.read(buffer.data(), size))
        {
            throw std::runtime_error("Could not read file: " + path);
        }

        return buffer;
    }

    std::unordered_set<uint32_t> CreateSupportedTypeSet(const webifc::schema::IfcSchemaManager &schemaManager)
    {
        std::unordered_set<uint32_t> types;
        const char *names[] =
        {
            "IFCEXTRUDEDAREASOLID",
            "IFCREVOLVEDAREASOLID",
            "IFCSURFACECURVESWEPTAREASOLID",
            "IFCFIXEDREFERENCESWEPTAREASOLID",
            "IFCSWEPTDISKSOLID",
            "IFCBSPLINESURFACE",
            "IFCBSPLINESURFACEWITHKNOTS",
            "IFCRATIONALBSPLINESURFACEWITHKNOTS",
            "IFCADVANCEDFACE",
            "IFCBOOLEANRESULT",
            "IFCBOOLEANCLIPPINGRESULT",
        };

        for (const auto *name : names)
        {
            auto typeCode = schemaManager.IfcTypeToTypeCode(name);
            if (typeCode != 0)
            {
                types.insert(typeCode);
            }
        }

        return types;
    }

    struct MeshMetric
    {
        uint32_t expressId = 0;
        uint32_t triangleCount = 0;
    };

    std::vector<uint32_t> ParseIdList(const std::string &csv)
    {
        std::vector<uint32_t> ids;
        std::stringstream ss(csv);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            if (item.empty())
            {
                continue;
            }

            const auto value = static_cast<uint32_t>(std::stoul(item));
            ids.push_back(value);
        }

        return ids;
    }
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: web-ifc-parity <path-to-ifc>" << std::endl;
        return 1;
    }

    const std::string inputPath = argv[1];
    std::vector<uint32_t> debugExpressIds;
    if (argc >= 3)
    {
        debugExpressIds = ParseIdList(argv[2]);
    }

    try
    {
        auto content = ReadFile(inputPath);

        const uint32_t tapeSize = 64U * 1024U * 1024U;
        const uint32_t memoryLimit = 2U * 1024U * 1024U * 1024U;
        const uint16_t lineWriterBuffer = 10'000U;

        webifc::schema::IfcSchemaManager schemaManager;
        webifc::parsing::IfcLoader loader(tapeSize, memoryLimit, lineWriterBuffer, schemaManager);
        loader.LoadFile([&](char *dest, size_t sourceOffset, size_t destSize)
                        {
                            if (sourceOffset >= content.size())
                            {
                                return static_cast<uint32_t>(0);
                            }

                            const auto length = std::min(content.size() - sourceOffset, destSize);
                            memcpy(dest, &content[sourceOffset], length);
                            return static_cast<uint32_t>(length);
                        });

        const uint16_t circleSegments = 12;
        const bool coordinateToOrigin = true;
        const double tolerancePlaneIntersection = 1.0E-04;
        const double tolerancePlaneDeviation = 1.0E-04;
        const double toleranceBackDeviationDistance = 1.0E-04;
        const double toleranceInsideOutsidePerimeter = 1.0E-10;
        const double toleranceScalarEquality = 1.0E-04;
        const double planeRefitIterations = 1;
        const double booleanUnionThreshold = 150;

        webifc::geometry::IfcGeometryProcessor geometryProcessor(
            loader,
            schemaManager,
            circleSegments,
            coordinateToOrigin,
            tolerancePlaneIntersection,
            tolerancePlaneDeviation,
            toleranceBackDeviationDistance,
            toleranceInsideOutsidePerimeter,
            toleranceScalarEquality,
            planeRefitIterations,
            booleanUnionThreshold);

        webifc::geometry::IfcGeometryLoader geometryLoader(
            loader,
            schemaManager,
            circleSegments,
            tolerancePlaneIntersection,
            tolerancePlaneDeviation,
            toleranceBackDeviationDistance,
            toleranceInsideOutsidePerimeter,
            toleranceScalarEquality,
            planeRefitIterations,
            booleanUnionThreshold);

        const auto supportedTypes = CreateSupportedTypeSet(schemaManager);
        std::vector<MeshMetric> meshes;
        meshes.reserve(loader.GetAllLines().size());

        uint64_t totalTriangles = 0;

        for (auto expressId : loader.GetAllLines())
        {
            auto lineType = loader.GetLineType(expressId);
            if (supportedTypes.find(lineType) == supportedTypes.end())
            {
                continue;
            }

            auto flatMesh = geometryProcessor.GetFlatMesh(expressId, false);
            uint32_t triangles = 0;

            for (const auto &placed : flatMesh.geometries)
            {
                auto &geometry = geometryProcessor.GetGeometry(placed.geometryExpressID);
                triangles += geometry.GetIndexDataSize() / 3U;
            }

            if (triangles == 0)
            {
                continue;
            }

            meshes.push_back({expressId, triangles});
            totalTriangles += triangles;
        }

        std::sort(
            meshes.begin(),
            meshes.end(),
            [](const MeshMetric &left, const MeshMetric &right)
            {
                return left.expressId < right.expressId;
            });

        if (!debugExpressIds.empty())
        {
            const auto extrudedType = schemaManager.IfcTypeToTypeCode("IFCEXTRUDEDAREASOLID");
            for (const auto expressId : debugExpressIds)
            {
                const auto lineType = loader.GetLineType(expressId);
                if (lineType != extrudedType)
                {
                    const auto curve = geometryLoader.GetCurve(expressId, 2);
                    if (curve.points.empty())
                    {
                        std::cerr << "[debug] #" << expressId << " unsupported type" << std::endl;
                        continue;
                    }

                    std::cerr << "[curve-debug] #" << expressId
                              << " points=" << curve.points.size();
                    for (size_t i = 0; i < curve.points.size(); i++)
                    {
                        const auto &p = curve.points[i];
                        std::cerr << std::setprecision(17);
                        std::cerr << " p" << i << "="
                                  << p.x << "," << p.y << "," << p.z;
                    }
                    std::cerr
                              << std::endl;
                    continue;
                }

                loader.MoveToArgumentOffset(expressId, 0);
                const auto profileId = loader.GetRefArgument();
                const auto profile = geometryLoader.GetProfile(profileId);
                const bool outerClosed = !profile.curve.points.empty() &&
                    glm::length(profile.curve.points.front() - profile.curve.points.back()) <= 1e-8;
                const auto projection = BestProjection(profile.curve.points);
                const auto areaXY = Area2D(profile.curve.points, Projection::XY);
                const auto areaXZ = Area2D(profile.curve.points, Projection::XZ);
                const auto areaYZ = Area2D(profile.curve.points, Projection::YZ);
                std::cerr << "[profile-debug] solid #" << expressId
                          << " profile #" << profileId
                          << " outerPoints=" << profile.curve.points.size()
                          << " outerClosed=" << (outerClosed ? "T" : "F")
                          << " projection=" << ProjectionName(projection)
                          << " areaXY=" << areaXY
                          << " areaXZ=" << areaXZ
                          << " areaYZ=" << areaYZ
                          << " holes=" << profile.holes.size();
                for (size_t i = 0; i < profile.holes.size(); i++)
                {
                    const auto& hole = profile.holes[i];
                    const bool holeClosed = !hole.points.empty() &&
                        glm::length(hole.points.front() - hole.points.back()) <= 1e-8;
                    std::cerr << " hole" << i << "Points=" << hole.points.size()
                              << " hole" << i << "Closed=" << (holeClosed ? "T" : "F");
                }
                std::cerr << std::endl;
            }
        }

        std::cout << "{";
        std::cout << "\"meshCount\":" << meshes.size() << ",";
        std::cout << "\"triangleCount\":" << totalTriangles << ",";
        std::cout << "\"meshes\":[";
        for (size_t i = 0; i < meshes.size(); i++)
        {
            const auto &mesh = meshes[i];
            if (i > 0)
            {
                std::cout << ",";
            }

            std::cout << "{";
            std::cout << "\"expressId\":" << mesh.expressId << ",";
            std::cout << "\"triangleCount\":" << mesh.triangleCount;
            std::cout << "}";
        }

        std::cout << "]}";
        std::cout << std::endl;
        return 0;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "web-ifc-parity failed: " << ex.what() << std::endl;
        return 2;
    }
}
