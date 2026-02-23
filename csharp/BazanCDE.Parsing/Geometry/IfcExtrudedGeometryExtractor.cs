using System.Numerics;
using BazanCDE.Parsing.Operations;

namespace BazanCDE.Parsing;

public static class IfcModelGeometryAnalyzer
{
    private const uint TapeSize = 64 * 1024 * 1024;
    private const ulong MemoryLimit = 2UL * 1024 * 1024 * 1024;
    private const uint LineWriterBuffer = 10_000;

    public static IfcParsedModel Analyze(byte[] modelBytes, int maxMeshes = 1_500, int maxTriangles = 800_000)
    {
        using var stream = new MemoryStream(modelBytes, writable: false);
        return Analyze(stream, maxMeshes, maxTriangles);
    }

    public static IfcParsedModel Analyze(Stream modelStream, int maxMeshes = 1_500, int maxTriangles = 800_000)
    {
        modelStream.Position = 0;
        var schemaManager = new DefaultIfcSchemaManager();

        using var loader = new IfcLoader(
            tapeSize: TapeSize,
            memoryLimit: MemoryLimit,
            lineWriterBuffer: LineWriterBuffer,
            schemaManager: schemaManager
        );

        loader.LoadFile(modelStream);
        var extractor = new IfcExtrudedGeometryExtractor(loader, schemaManager);
        return extractor.Extract(maxMeshes, maxTriangles);
    }
}

internal sealed class IfcExtrudedGeometryExtractor
{
    private const int DefaultCircleSegments = 12;

    private readonly IfcLoader _loader;

    private readonly uint _ifcCartesianPointType;
    private readonly uint _ifcDirectionType;
    private readonly uint _ifcAxis2Placement3DType;
    private readonly uint _ifcAxis2Placement2DType;
    private readonly uint _ifcPolylineType;
    private readonly uint _ifcPolyLoopType;
    private readonly uint _ifcIndexedPolyCurveType;
    private readonly uint _ifcCompositeCurveType;
    private readonly uint _ifcCompositeCurveSegmentType;
    private readonly uint _ifcTrimmedCurveType;
    private readonly uint _ifcCircleType;
    private readonly uint _ifcEllipseType;
    private readonly uint _ifcBsplineCurveType;
    private readonly uint _ifcBsplineCurveWithKnotsType;
    private readonly uint _ifcCartesianPointList2DType;
    private readonly uint _ifcCartesianPointList3DType;
    private readonly uint _ifcArbitraryClosedProfileDefType;
    private readonly uint _ifcArbitraryProfileDefWithVoidsType;
    private readonly uint _ifcDerivedProfileDefType;
    private readonly uint _ifcRectangleProfileDefType;
    private readonly uint _ifcCartesianTransformationOperator2DType;
    private readonly uint _ifcCartesianTransformationOperator2DNonUniformType;
    private readonly uint _ifcExtrudedAreaSolidType;
    private readonly uint _ifcRevolvedAreaSolidType;
    private readonly uint _ifcSurfaceCurveSweptAreaSolidType;
    private readonly uint _ifcFixedReferenceSweptAreaSolidType;
    private readonly uint _ifcSweptDiskSolidType;
    private readonly uint _ifcBsplineSurfaceType;
    private readonly uint _ifcBsplineSurfaceWithKnotsType;
    private readonly uint _ifcRationalBsplineSurfaceWithKnotsType;
    private readonly uint _ifcAdvancedFaceType;
    private readonly uint _ifcFaceOuterBoundType;
    private readonly uint _ifcFaceBoundType;
    private readonly uint _ifcBooleanResultType;
    private readonly uint _ifcBooleanClippingResultType;
    private readonly uint _ifcHalfSpaceSolidType;
    private readonly uint _ifcAxis1PlacementType;

    private readonly Dictionary<uint, Vector3> _pointCache = new();
    private readonly Dictionary<uint, Vector3> _directionCache = new();
    private readonly Dictionary<uint, Placement3D> _axis3Cache = new();
    private readonly Dictionary<uint, Axis1Placement> _axis1Cache = new();
    private readonly Dictionary<uint, Placement2D> _axis2Cache = new();
    private readonly Dictionary<uint, List<Vector2>> _curve2DCache = new();
    private readonly Dictionary<uint, List<Vector3>> _curve3DCache = new();

    public IfcExtrudedGeometryExtractor(IfcLoader loader, IIfcSchemaManager schemaManager)
    {
        _loader = loader;

        _ifcCartesianPointType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANPOINT");
        _ifcDirectionType = schemaManager.IfcTypeToTypeCode("IFCDIRECTION");
        _ifcAxis2Placement3DType = schemaManager.IfcTypeToTypeCode("IFCAXIS2PLACEMENT3D");
        _ifcAxis2Placement2DType = schemaManager.IfcTypeToTypeCode("IFCAXIS2PLACEMENT2D");
        _ifcPolylineType = schemaManager.IfcTypeToTypeCode("IFCPOLYLINE");
        _ifcPolyLoopType = schemaManager.IfcTypeToTypeCode("IFCPOLYLOOP");
        _ifcIndexedPolyCurveType = schemaManager.IfcTypeToTypeCode("IFCINDEXEDPOLYCURVE");
        _ifcCompositeCurveType = schemaManager.IfcTypeToTypeCode("IFCCOMPOSITECURVE");
        _ifcCompositeCurveSegmentType = schemaManager.IfcTypeToTypeCode("IFCCOMPOSITECURVESEGMENT");
        _ifcTrimmedCurveType = schemaManager.IfcTypeToTypeCode("IFCTRIMMEDCURVE");
        _ifcCircleType = schemaManager.IfcTypeToTypeCode("IFCCIRCLE");
        _ifcEllipseType = schemaManager.IfcTypeToTypeCode("IFCELLIPSE");
        _ifcBsplineCurveType = schemaManager.IfcTypeToTypeCode("IFCBSPLINECURVE");
        _ifcBsplineCurveWithKnotsType = schemaManager.IfcTypeToTypeCode("IFCBSPLINECURVEWITHKNOTS");
        _ifcCartesianPointList2DType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANPOINTLIST2D");
        _ifcCartesianPointList3DType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANPOINTLIST3D");
        _ifcArbitraryClosedProfileDefType = schemaManager.IfcTypeToTypeCode("IFCARBITRARYCLOSEDPROFILEDEF");
        _ifcArbitraryProfileDefWithVoidsType = schemaManager.IfcTypeToTypeCode("IFCARBITRARYPROFILEDEFWITHVOIDS");
        _ifcDerivedProfileDefType = schemaManager.IfcTypeToTypeCode("IFCDERIVEDPROFILEDEF");
        _ifcRectangleProfileDefType = schemaManager.IfcTypeToTypeCode("IFCRECTANGLEPROFILEDEF");
        _ifcCartesianTransformationOperator2DType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANTRANSFORMATIONOPERATOR2D");
        _ifcCartesianTransformationOperator2DNonUniformType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM");
        _ifcExtrudedAreaSolidType = schemaManager.IfcTypeToTypeCode("IFCEXTRUDEDAREASOLID");
        _ifcRevolvedAreaSolidType = schemaManager.IfcTypeToTypeCode("IFCREVOLVEDAREASOLID");
        _ifcSurfaceCurveSweptAreaSolidType = schemaManager.IfcTypeToTypeCode("IFCSURFACECURVESWEPTAREASOLID");
        _ifcFixedReferenceSweptAreaSolidType = schemaManager.IfcTypeToTypeCode("IFCFIXEDREFERENCESWEPTAREASOLID");
        _ifcSweptDiskSolidType = schemaManager.IfcTypeToTypeCode("IFCSWEPTDISKSOLID");
        _ifcBsplineSurfaceType = schemaManager.IfcTypeToTypeCode("IFCBSPLINESURFACE");
        _ifcBsplineSurfaceWithKnotsType = schemaManager.IfcTypeToTypeCode("IFCBSPLINESURFACEWITHKNOTS");
        _ifcRationalBsplineSurfaceWithKnotsType = schemaManager.IfcTypeToTypeCode("IFCRATIONALBSPLINESURFACEWITHKNOTS");
        _ifcAdvancedFaceType = schemaManager.IfcTypeToTypeCode("IFCADVANCEDFACE");
        _ifcFaceOuterBoundType = schemaManager.IfcTypeToTypeCode("IFCFACEOUTERBOUND");
        _ifcFaceBoundType = schemaManager.IfcTypeToTypeCode("IFCFACEBOUND");
        _ifcBooleanResultType = schemaManager.IfcTypeToTypeCode("IFCBOOLEANRESULT");
        _ifcBooleanClippingResultType = schemaManager.IfcTypeToTypeCode("IFCBOOLEANCLIPPINGRESULT");
        _ifcHalfSpaceSolidType = schemaManager.IfcTypeToTypeCode("IFCHALFSPACESOLID");
        _ifcAxis1PlacementType = schemaManager.IfcTypeToTypeCode("IFCAXIS1PLACEMENT");
    }

    public IfcParsedModel Extract(int maxMeshes, int maxTriangles)
    {
        maxMeshes = Math.Max(1, maxMeshes);
        maxTriangles = Math.Max(1, maxTriangles);

        var meshes = new List<IfcParsedMesh>(Math.Min(maxMeshes, 512));
        var triangleCount = 0;

        foreach (var expressId in _loader.GetAllLines())
        {
            var lineType = _loader.GetLineType(expressId);
            if (!IsSupportedSolidType(lineType))
            {
                continue;
            }

            if (meshes.Count >= maxMeshes || triangleCount >= maxTriangles)
            {
                break;
            }

            if (!TryBuildMesh(expressId, out var mesh, out var trianglesInMesh))
            {
                continue;
            }

            if (triangleCount + trianglesInMesh > maxTriangles)
            {
                break;
            }

            meshes.Add(mesh);
            triangleCount += trianglesInMesh;
        }

        return new IfcParsedModel(
            MeshCount: meshes.Count,
            TriangleCount: triangleCount,
            Meshes: meshes
        );
    }

    public bool TryBuildMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        return TryBuildMeshInternal(expressId, new HashSet<uint>(), out mesh, out triangleCount);
    }

    private bool TryBuildMeshInternal(uint expressId, HashSet<uint> visiting, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        if (!visiting.Add(expressId))
        {
            return false;
        }

        var lineType = _loader.GetLineType(expressId);
        var built = lineType switch
        {
            var t when t == _ifcExtrudedAreaSolidType => TryBuildExtrudedMesh(expressId, out mesh, out triangleCount),
            var t when t == _ifcRevolvedAreaSolidType => TryBuildRevolvedMesh(expressId, out mesh, out triangleCount),
            var t when t == _ifcSurfaceCurveSweptAreaSolidType => TryBuildSurfaceCurveSweptMesh(expressId, out mesh, out triangleCount),
            var t when t == _ifcFixedReferenceSweptAreaSolidType => TryBuildFixedReferenceSweptMesh(expressId, out mesh, out triangleCount),
            var t when t == _ifcSweptDiskSolidType => TryBuildSweptDiskMesh(expressId, out mesh, out triangleCount),
            var t when t == _ifcBsplineSurfaceType => TryBuildBsplineSurfaceMesh(expressId, hasKnots: false, hasWeights: false, out mesh, out triangleCount),
            var t when t == _ifcBsplineSurfaceWithKnotsType => TryBuildBsplineSurfaceMesh(expressId, hasKnots: true, hasWeights: false, out mesh, out triangleCount),
            var t when t == _ifcRationalBsplineSurfaceWithKnotsType => TryBuildBsplineSurfaceMesh(expressId, hasKnots: true, hasWeights: true, out mesh, out triangleCount),
            var t when t == _ifcAdvancedFaceType => TryBuildAdvancedFaceMesh(expressId, out mesh, out triangleCount),
            var t when t == _ifcBooleanResultType => TryBuildBooleanResultMesh(expressId, visiting, out mesh, out triangleCount),
            var t when t == _ifcBooleanClippingResultType => TryBuildBooleanClippingMesh(expressId, visiting, out mesh, out triangleCount),
            _ => false
        };

        visiting.Remove(expressId);
        return built;
    }

    private bool IsSupportedSolidType(uint lineType)
    {
        return lineType == _ifcExtrudedAreaSolidType
            || lineType == _ifcRevolvedAreaSolidType
            || lineType == _ifcSurfaceCurveSweptAreaSolidType
            || lineType == _ifcFixedReferenceSweptAreaSolidType
            || lineType == _ifcSweptDiskSolidType
            || lineType == _ifcBsplineSurfaceType
            || lineType == _ifcBsplineSurfaceWithKnotsType
            || lineType == _ifcRationalBsplineSurfaceWithKnotsType
            || lineType == _ifcAdvancedFaceType
            || lineType == _ifcBooleanResultType
            || lineType == _ifcBooleanClippingResultType;
    }

    private bool TryBuildExtrudedMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var profileRef = ReadOptionalRefArg(expressId, 0);
        if (profileRef == 0 || !TryGetProfileLoops2D(profileRef, out var loops) || loops.Count == 0)
        {
            return false;
        }

        var polygon = NormalizePolygon(loops[0]);
        if (polygon.Count < 3)
        {
            return false;
        }

        var capTriangles = TriangulatePolygon(polygon);
        if (capTriangles.Count == 0)
        {
            return false;
        }

        var positionRef = ReadOptionalRefArg(expressId, 1);
        var directionRef = ReadOptionalRefArg(expressId, 2);
        var depth = ReadNumericArg(expressId, 3, defaultValue: 0);
        if (depth <= 1e-6)
        {
            return false;
        }

        var placement = positionRef != 0 && TryGetAxis2Placement3D(positionRef, out var position)
            ? position
            : Placement3D.Identity;

        var localDirection = directionRef != 0 && TryGetDirection(directionRef, out var extrusionDirection)
            ? extrusionDirection
            : Vector3.UnitZ;
        var worldDirection = placement.TransformDirection(localDirection);
        if (worldDirection.LengthSquared() < 1e-8f)
        {
            worldDirection = placement.AxisZ;
        }

        worldDirection = Vector3.Normalize(worldDirection) * (float)depth;

        var operationProfiles = new List<IReadOnlyList<Vector3>>(loops.Count);
        for (var i = 0; i < loops.Count; i++)
        {
            var normalizedLoop = NormalizePolygon(loops[i]);
            if (normalizedLoop.Count < 3)
            {
                continue;
            }

            var profileLoop = new List<Vector3>(normalizedLoop.Count);
            for (var j = 0; j < normalizedLoop.Count; j++)
            {
                var p2 = normalizedLoop[j];
                profileLoop.Add(placement.TransformPoint(new Vector3(p2.X, p2.Y, 0)));
            }

            operationProfiles.Add(profileLoop);
        }

        if (operationProfiles.Count > 0)
        {
            var geom = GeometryOps.Extrude(
                operationProfiles,
                worldDirection,
                worldDirection.Length(),
                worldDirection,
                placement.Origin);

            if (geom.NumFaces > 0)
            {
                mesh = BuildParsedMeshFromGeometry(expressId, geom, CreateColor(expressId));
                triangleCount = mesh.Indices.Length / 3;
                return triangleCount > 0;
            }
        }

        var vertexCount = polygon.Count;
        var positions = new float[vertexCount * 2 * 3];

        for (var i = 0; i < vertexCount; i++)
        {
            var point2 = polygon[i];
            var basePoint = placement.TransformPoint(new Vector3(point2.X, point2.Y, 0));
            var topPoint = basePoint + worldDirection;

            var baseOffset = i * 3;
            positions[baseOffset] = basePoint.X;
            positions[baseOffset + 1] = basePoint.Y;
            positions[baseOffset + 2] = basePoint.Z;

            var topOffset = (vertexCount + i) * 3;
            positions[topOffset] = topPoint.X;
            positions[topOffset + 1] = topPoint.Y;
            positions[topOffset + 2] = topPoint.Z;
        }

        var indices = BuildExtrusionIndices(vertexCount, capTriangles);
        if (indices.Length == 0)
        {
            return false;
        }

        triangleCount = indices.Length / 3;
        mesh = new IfcParsedMesh(
            ExpressId: expressId,
            Positions: positions,
            Indices: indices,
            Color: CreateColor(expressId)
        );
        return true;
    }

    private bool TryBuildRevolvedMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var profileRef = ReadOptionalRefArg(expressId, 0);
        var placementRef = ReadOptionalRefArg(expressId, 1);
        var axis1PlacementRef = ReadOptionalRefArg(expressId, 2);
        var angleRaw = ReadNumericArg(expressId, 3, defaultValue: 0);

        if (profileRef == 0 || axis1PlacementRef == 0 || angleRaw <= 1e-8)
        {
            return false;
        }

        if (!TryGetProfileOutline2D(profileRef, out var outline))
        {
            return false;
        }

        if (!TryGetAxis1Placement(axis1PlacementRef, out var axis1))
        {
            return false;
        }

        // Swept profile is 2D; for revolution we map to local XZ plane.
        var profile3D = outline.Select(p => new Vector3(p.X, 0, p.Y)).ToList();
        if (profile3D.Count < 3)
        {
            return false;
        }

        var angleDegrees = ConvertAngleToDegrees(angleRaw);
        var segments = Math.Max(8, (int)Math.Ceiling(Math.Abs(angleDegrees) / 12.0) + 1);

        var geom = GeometryOps.Revolution(axis1.ToMatrix(), 0, angleDegrees, profile3D, segments);
        if (geom.NumFaces == 0)
        {
            return false;
        }

        if (placementRef != 0 && TryGetAxis2Placement3D(placementRef, out var placement))
        {
            ApplyPlacementToGeometry(geom, placement);
        }

        var color = CreateColor(expressId);
        mesh = BuildParsedMeshFromGeometry(expressId, geom, color);
        triangleCount = mesh.Indices.Length / 3;
        return triangleCount > 0;
    }

    private bool TryBuildSurfaceCurveSweptMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var profileRef = ReadOptionalRefArg(expressId, 0);
        var placementRef = ReadOptionalRefArg(expressId, 1);
        var directrixRef = ReadOptionalRefArg(expressId, 2);
        var surfaceRef = ReadOptionalRefArg(expressId, 5);

        if (profileRef == 0 || directrixRef == 0)
        {
            return false;
        }

        if (!TryGetProfileOutline2D(profileRef, out var outline) || !TryGetCurve3D(directrixRef, out var directrix))
        {
            return false;
        }

        var profile3D = outline.Select(p => new Vector3(p.X, p.Y, 0)).ToList();
        profile3D.Reverse();

        var closed = IsCurveClosedByDefinition(directrixRef);
        var initialNormal = Vector3.UnitZ;

        if (surfaceRef != 0)
        {
            if (TryGetDirection(surfaceRef, out var dir))
            {
                initialNormal = dir;
            }
            else if (TryGetPoint(surfaceRef, out var point))
            {
                initialNormal = SafeNormalize(point, Vector3.UnitZ);
            }
        }

        var geom = GeometryOps.SweepFunction(
            scaling: 1.0,
            closed: closed,
            profilePoints: profile3D,
            directrix: directrix,
            initialDirectrixNormal: initialNormal,
            rotate90: true,
            optimize: false);

        if (geom.NumFaces == 0)
        {
            return false;
        }

        if (placementRef != 0 && TryGetAxis2Placement3D(placementRef, out var placement))
        {
            ApplyPlacementToGeometry(geom, placement);
        }

        var color = CreateColor(expressId);
        mesh = BuildParsedMeshFromGeometry(expressId, geom, color);
        triangleCount = mesh.Indices.Length / 3;
        return triangleCount > 0;
    }

    private bool TryBuildFixedReferenceSweptMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var profileRef = ReadOptionalRefArg(expressId, 0);
        var placementRef = ReadOptionalRefArg(expressId, 1);
        var directrixRef = ReadOptionalRefArg(expressId, 2);
        var fixedReferenceRef = ReadOptionalRefArg(expressId, 3);

        if (profileRef == 0 || directrixRef == 0)
        {
            return false;
        }

        if (!TryGetProfileOutline2D(profileRef, out var outline) || !TryGetCurve3D(directrixRef, out var directrix))
        {
            return false;
        }

        var profile3D = outline.Select(p => new Vector3(p.X, p.Y, 0)).ToList();
        var closed = IsCurveClosedByDefinition(directrixRef);

        Vector3 fixedReference = Vector3.UnitY;
        if (fixedReferenceRef != 0)
        {
            if (!TryGetDirection(fixedReferenceRef, out fixedReference) && TryGetPoint(fixedReferenceRef, out var point))
            {
                fixedReference = point;
            }
        }

        fixedReference = SafeNormalize(fixedReference, Vector3.UnitY);

        var geom = GeometryOps.SweepFunction(
            scaling: 1.0,
            closed: closed,
            profilePoints: profile3D,
            directrix: directrix,
            initialDirectrixNormal: fixedReference,
            rotate90: false,
            optimize: false);

        if (geom.NumFaces == 0)
        {
            return false;
        }

        if (placementRef != 0 && TryGetAxis2Placement3D(placementRef, out var placement))
        {
            ApplyPlacementToGeometry(geom, placement);
        }

        var color = CreateColor(expressId);
        mesh = BuildParsedMeshFromGeometry(expressId, geom, color);
        triangleCount = mesh.Indices.Length / 3;
        return triangleCount > 0;
    }

    private bool TryBuildSweptDiskMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var directrixRef = ReadOptionalRefArg(expressId, 0);
        var radius = ReadNumericArg(expressId, 1, defaultValue: 0);
        if (directrixRef == 0 || radius <= 1e-8)
        {
            return false;
        }

        if (!TryGetCurve3D(directrixRef, out var directrix))
        {
            return false;
        }

        var circle = GeometryOps.GetEllipseCurve(1, 1, 16).Points;
        var geom = GeometryOps.SweepCircular(
            scaling: 1.0,
            closed: false,
            profilePoints: circle,
            radius: radius,
            directrix: directrix,
            initialDirectrixNormal: Vector3.UnitY,
            rotate90: false);

        if (geom.NumFaces == 0)
        {
            return false;
        }

        var color = CreateColor(expressId);
        mesh = BuildParsedMeshFromGeometry(expressId, geom, color);
        triangleCount = mesh.Indices.Length / 3;
        return triangleCount > 0;
    }

    private bool TryBuildBsplineSurfaceMesh(
        uint expressId,
        bool hasKnots,
        bool hasWeights,
        out IfcParsedMesh mesh,
        out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var degreeU = (int)Math.Round(ReadNumericArg(expressId, 0, defaultValue: 3));
        var degreeV = (int)Math.Round(ReadNumericArg(expressId, 1, defaultValue: 3));
        degreeU = Math.Max(1, degreeU);
        degreeV = Math.Max(1, degreeV);

        if (!TryGetControlGrid3D(expressId, 2, out var controls))
        {
            return false;
        }

        var rows = controls.Count;
        var cols = controls[0].Count;
        if (rows < degreeU + 1 || cols < degreeV + 1)
        {
            return false;
        }

        var weights = new List<List<double>>(rows);
        for (var r = 0; r < rows; r++)
        {
            var row = new List<double>(cols);
            for (var c = 0; c < cols; c++)
            {
                row.Add(1.0);
            }

            weights.Add(row);
        }

        if (hasWeights)
        {
            if (!TryGetWeightGrid(expressId, 12, rows, cols, out var weightGrid))
            {
                return false;
            }

            weights = weightGrid;
        }

        var knotsU = hasKnots && TryReadExpandedKnots(expressId, 7, 9, out var ku)
            ? ku
            : BuildUniformOpenKnotVector(degreeU, rows);
        var knotsV = hasKnots && TryReadExpandedKnots(expressId, 8, 10, out var kv)
            ? kv
            : BuildUniformOpenKnotVector(degreeV, cols);

        if (knotsU.Count != rows + degreeU + 1)
        {
            knotsU = BuildUniformOpenKnotVector(degreeU, rows);
        }

        if (knotsV.Count != cols + degreeV + 1)
        {
            knotsV = BuildUniformOpenKnotVector(degreeV, cols);
        }

        var sampleU = Math.Clamp(rows * 4, 8, 48);
        var sampleV = Math.Clamp(cols * 4, 8, 48);
        var vertices = new List<Vector3>(sampleU * sampleV);
        var indices = new List<uint>((sampleU - 1) * (sampleV - 1) * 6);

        var startU = knotsU[degreeU];
        var endU = knotsU[rows];
        var startV = knotsV[degreeV];
        var endV = knotsV[cols];
        if (Math.Abs(endU - startU) < 1e-10)
        {
            endU = startU + 1;
        }

        if (Math.Abs(endV - startV) < 1e-10)
        {
            endV = startV + 1;
        }

        for (var uIndex = 0; uIndex < sampleU; uIndex++)
        {
            var tu = uIndex / (double)(sampleU - 1);
            var u = startU + (endU - startU) * tu;

            for (var vIndex = 0; vIndex < sampleV; vIndex++)
            {
                var tv = vIndex / (double)(sampleV - 1);
                var v = startV + (endV - startV) * tv;
                vertices.Add(EvaluateNurbsSurfacePoint(controls, weights, degreeU, degreeV, knotsU, knotsV, u, v));
            }
        }

        for (var u = 0; u < sampleU - 1; u++)
        {
            for (var v = 0; v < sampleV - 1; v++)
            {
                var a = (uint)(u * sampleV + v);
                var b = (uint)((u + 1) * sampleV + v);
                var c = (uint)((u + 1) * sampleV + v + 1);
                var d = (uint)(u * sampleV + v + 1);

                indices.Add(a);
                indices.Add(b);
                indices.Add(c);
                indices.Add(a);
                indices.Add(c);
                indices.Add(d);
            }
        }

        if (indices.Count == 0)
        {
            return false;
        }

        var positions = new float[vertices.Count * 3];
        for (var i = 0; i < vertices.Count; i++)
        {
            var o = i * 3;
            positions[o] = vertices[i].X;
            positions[o + 1] = vertices[i].Y;
            positions[o + 2] = vertices[i].Z;
        }

        mesh = new IfcParsedMesh(expressId, positions, indices.ToArray(), CreateColor(expressId));
        triangleCount = mesh.Indices.Length / 3;
        return triangleCount > 0;
    }

    private bool TryBuildAdvancedFaceMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var boundRefs = ReadRefSetArg(expressId, 0);
        var surfaceRef = ReadOptionalRefArg(expressId, 1);
        if (surfaceRef == 0 || boundRefs.Count == 0)
        {
            return false;
        }

        if (!TryGetBsplineSurfaceDefinition(surfaceRef, out var surface))
        {
            return false;
        }

        var loops = new List<(bool IsOuter, List<Vector2> UvLoop)>();
        foreach (var boundRef in boundRefs)
        {
            var boundType = _loader.GetLineType(boundRef);
            if (boundType != _ifcFaceOuterBoundType && boundType != _ifcFaceBoundType)
            {
                continue;
            }

            var loopRef = ReadOptionalRefArg(boundRef, 0);
            if (loopRef == 0 || !TryGetCurve3D(loopRef, out var points3D) || points3D.Count < 3)
            {
                continue;
            }

            var orientation = ReadLogicalArg(boundRef, 1, defaultValue: true);
            if (!orientation)
            {
                points3D.Reverse();
            }

            var uvLoop = new List<Vector2>(points3D.Count);
            for (var i = 0; i < points3D.Count; i++)
            {
                uvLoop.Add(InverseEvaluateNurbsSurfacePoint(surface, points3D[i]));
            }

            uvLoop = NormalizePolygon(uvLoop);
            if (uvLoop.Count >= 3)
            {
                loops.Add((boundType == _ifcFaceOuterBoundType, uvLoop));
            }
        }

        if (loops.Count == 0)
        {
            return false;
        }

        List<Vector2> outer = loops.FirstOrDefault(l => l.IsOuter).UvLoop;
        if (outer.Count == 0)
        {
            outer = loops[0].UvLoop;
        }

        var holes = loops.Where(l => !ReferenceEquals(l.UvLoop, outer)).Select(l => l.UvLoop).ToList();
        if (holes.Count == 0 && loops.Count > 1)
        {
            holes = loops.Skip(1).Select(l => l.UvLoop).ToList();
        }

        var sampleU = Math.Clamp(surface.ControlPoints.Count * 6, 12, 64);
        var sampleV = Math.Clamp(surface.ControlPoints[0].Count * 6, 12, 64);

        var vertices = new List<Vector3>(sampleU * sampleV);
        for (var uIndex = 0; uIndex < sampleU; uIndex++)
        {
            var tu = uIndex / (double)(sampleU - 1);
            var u = surface.RangeU.X + (surface.RangeU.Y - surface.RangeU.X) * tu;
            for (var vIndex = 0; vIndex < sampleV; vIndex++)
            {
                var tv = vIndex / (double)(sampleV - 1);
                var v = surface.RangeV.X + (surface.RangeV.Y - surface.RangeV.X) * tv;
                vertices.Add(EvaluateNurbsSurfacePoint(surface.ControlPoints, surface.Weights, surface.DegreeU, surface.DegreeV, surface.KnotsU, surface.KnotsV, u, v));
            }
        }

        var indices = new List<uint>((sampleU - 1) * (sampleV - 1) * 6);
        for (var u = 0; u < sampleU - 1; u++)
        {
            for (var v = 0; v < sampleV - 1; v++)
            {
                var ua = surface.RangeU.X + (surface.RangeU.Y - surface.RangeU.X) * (u / (double)(sampleU - 1));
                var ub = surface.RangeU.X + (surface.RangeU.Y - surface.RangeU.X) * ((u + 1) / (double)(sampleU - 1));
                var va = surface.RangeV.X + (surface.RangeV.Y - surface.RangeV.X) * (v / (double)(sampleV - 1));
                var vb = surface.RangeV.X + (surface.RangeV.Y - surface.RangeV.X) * ((v + 1) / (double)(sampleV - 1));

                var c1 = new Vector2((float)((ua + ub + ub) / 3.0), (float)((va + va + vb) / 3.0));
                var c2 = new Vector2((float)((ua + ub + ua) / 3.0), (float)((va + vb + vb) / 3.0));

                var a = (uint)(u * sampleV + v);
                var b = (uint)((u + 1) * sampleV + v);
                var c = (uint)((u + 1) * sampleV + v + 1);
                var d = (uint)(u * sampleV + v + 1);

                if (IsInsideTrimmedDomain(c1, outer, holes))
                {
                    indices.Add(a);
                    indices.Add(b);
                    indices.Add(c);
                }

                if (IsInsideTrimmedDomain(c2, outer, holes))
                {
                    indices.Add(a);
                    indices.Add(c);
                    indices.Add(d);
                }
            }
        }

        if (indices.Count == 0)
        {
            return false;
        }

        var positions = new float[vertices.Count * 3];
        for (var i = 0; i < vertices.Count; i++)
        {
            var o = i * 3;
            positions[o] = vertices[i].X;
            positions[o + 1] = vertices[i].Y;
            positions[o + 2] = vertices[i].Z;
        }

        mesh = new IfcParsedMesh(expressId, positions, indices.ToArray(), CreateColor(expressId));
        triangleCount = mesh.Indices.Length / 3;
        return triangleCount > 0;
    }

    private bool TryBuildBooleanResultMesh(uint expressId, HashSet<uint> visiting, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        _loader.MoveToLineArgument(expressId, 0);
        var tk = _loader.GetTokenType();
        _loader.StepBack();

        var op = "DIFFERENCE";
        if (tk is IfcTokenType.Enum or IfcTokenType.Label or IfcTokenType.String)
        {
            op = _loader.GetStringArgument();
        }

        var firstRef = ReadOptionalRefArg(expressId, 1);
        var secondRef = ReadOptionalRefArg(expressId, 2);
        if (firstRef == 0 || secondRef == 0)
        {
            return false;
        }

        return TryBuildBooleanMesh(expressId, firstRef, secondRef, op, visiting, out mesh, out triangleCount);
    }

    private bool TryBuildBooleanClippingMesh(uint expressId, HashSet<uint> visiting, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var firstRef = ReadOptionalRefArg(expressId, 1);
        var secondRef = ReadOptionalRefArg(expressId, 2);

        if (firstRef == 0 || secondRef == 0)
        {
            firstRef = ReadOptionalRefArg(expressId, 0);
            secondRef = ReadOptionalRefArg(expressId, 1);
        }

        if (firstRef == 0 || secondRef == 0)
        {
            return false;
        }

        if (_loader.GetLineType(secondRef) == _ifcHalfSpaceSolidType)
        {
            // IFC half-space clipping requires robust BRep clipping; keep first operand until full parity is ported.
            return TryBuildMeshInternal(firstRef, visiting, out mesh, out triangleCount);
        }

        return TryBuildBooleanMesh(expressId, firstRef, secondRef, "DIFFERENCE", visiting, out mesh, out triangleCount);
    }

    private bool TryBuildBooleanMesh(
        uint expressId,
        uint firstRef,
        uint secondRef,
        string op,
        HashSet<uint> visiting,
        out IfcParsedMesh mesh,
        out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        if (!TryBuildMeshInternal(firstRef, visiting, out var firstMesh, out _) ||
            !TryBuildMeshInternal(secondRef, visiting, out var secondMesh, out _))
        {
            return false;
        }

        var firstGeom = ToOperationsGeometry(firstMesh);
        var secondGeom = ToOperationsGeometry(secondMesh);

        var normalizedOp = NormalizeBooleanOperation(op);
        var resultGeom = GeometryOps.BoolProcess(firstGeom, [secondGeom], normalizedOp);
        if (resultGeom.NumFaces == 0)
        {
            return false;
        }

        mesh = BuildParsedMeshFromGeometry(expressId, resultGeom, firstMesh.Color);
        triangleCount = mesh.Indices.Length / 3;
        return triangleCount > 0;
    }

    private static string NormalizeBooleanOperation(string op)
    {
        var normalized = op.Trim();
        if (normalized.StartsWith(".", StringComparison.Ordinal) && normalized.EndsWith(".", StringComparison.Ordinal))
        {
            normalized = normalized.Trim('.');
        }

        return normalized.ToUpperInvariant() switch
        {
            "UNION" => "UNION",
            "INTERSECTION" => "INTERSECTION",
            _ => "DIFFERENCE"
        };
    }

    private bool TryGetProfileOutline2D(uint profileRef, out List<Vector2> points)
    {
        points = new List<Vector2>();
        if (!TryGetProfileLoops2D(profileRef, out var loops) || loops.Count == 0)
        {
            return false;
        }

        points = loops[0];
        return points.Count >= 3;
    }

    private bool TryGetProfileLoops2D(uint profileRef, out List<List<Vector2>> loops)
    {
        loops = new List<List<Vector2>>();
        var type = _loader.GetLineType(profileRef);

        if (type == _ifcRectangleProfileDefType)
        {
            var xDim = ReadNumericArg(profileRef, 3, defaultValue: 0);
            var yDim = ReadNumericArg(profileRef, 4, defaultValue: 0);
            if (xDim <= 1e-6 || yDim <= 1e-6)
            {
                return false;
            }

            var hx = (float)(xDim * 0.5);
            var hy = (float)(yDim * 0.5);
            var rectangle = new List<Vector2>
            {
                new(-hx, -hy),
                new(hx, -hy),
                new(hx, hy),
                new(-hx, hy)
            };

            var placementRef = ReadOptionalRefArg(profileRef, 2);
            if (placementRef != 0 && TryGetAxis2Placement2D(placementRef, out var placement))
            {
                for (var i = 0; i < rectangle.Count; i++)
                {
                    rectangle[i] = placement.TransformPoint(rectangle[i]);
                }
            }

            loops.Add(rectangle);
            return true;
        }

        if (type == _ifcArbitraryClosedProfileDefType || type == _ifcArbitraryProfileDefWithVoidsType)
        {
            var curveRef = ReadOptionalRefArg(profileRef, 2);
            if (curveRef == 0 || !TryGetCurve2D(curveRef, out var outer))
            {
                return false;
            }

            loops.Add(outer);

            if (type == _ifcArbitraryProfileDefWithVoidsType)
            {
                var innerRefs = ReadRefSetArg(profileRef, 3);
                foreach (var innerRef in innerRefs)
                {
                    if (TryGetCurve2D(innerRef, out var inner) && inner.Count >= 3)
                    {
                        loops.Add(inner);
                    }
                }
            }

            return loops.Count > 0;
        }

        if (type == _ifcDerivedProfileDefType)
        {
            var parentRef = ReadOptionalRefArg(profileRef, 2);
            if (parentRef == 0 || !TryGetProfileLoops2D(parentRef, out var parentLoops) || parentLoops.Count == 0)
            {
                return false;
            }

            var transformRef = ReadOptionalRefArg(profileRef, 3);
            if (transformRef != 0 && TryGetCartesianTransformation2D(transformRef, out var transform))
            {
                loops.Capacity = parentLoops.Count;
                for (var i = 0; i < parentLoops.Count; i++)
                {
                    var source = parentLoops[i];
                    var transformed = new List<Vector2>(source.Count);
                    for (var j = 0; j < source.Count; j++)
                    {
                        transformed.Add(transform.TransformPoint(source[j]));
                    }

                    loops.Add(transformed);
                }
            }
            else
            {
                loops.Capacity = parentLoops.Count;
                for (var i = 0; i < parentLoops.Count; i++)
                {
                    loops.Add(new List<Vector2>(parentLoops[i]));
                }
            }

            return loops.Count > 0;
        }

        return false;
    }

    private bool TryGetCartesianTransformation2D(uint transformRef, out Transform2D transform)
    {
        transform = Transform2D.Identity;
        var type = _loader.GetLineType(transformRef);
        if (type != _ifcCartesianTransformationOperator2DType &&
            type != _ifcCartesianTransformationOperator2DNonUniformType)
        {
            return false;
        }

        var axis1Ref = ReadOptionalRefArg(transformRef, 0);
        var axis2Ref = ReadOptionalRefArg(transformRef, 1);
        var originRef = ReadOptionalRefArg(transformRef, 2);
        var scaleX = ReadNumericArg(transformRef, 3, defaultValue: 1.0);
        var scaleY = type == _ifcCartesianTransformationOperator2DNonUniformType
            ? ReadNumericArg(transformRef, 4, defaultValue: scaleX)
            : scaleX;

        var axisX = Vector2.UnitX;
        if (axis1Ref != 0 && TryGetDirection(axis1Ref, out var axis1))
        {
            axisX = SafeNormalize(new Vector2(axis1.X, axis1.Y), Vector2.UnitX);
        }

        var axisY = new Vector2(-axisX.Y, axisX.X);
        if (axis2Ref != 0 && TryGetDirection(axis2Ref, out var axis2))
        {
            axisY = SafeNormalize(new Vector2(axis2.X, axis2.Y), Vector2.UnitY);
        }

        var origin = Vector2.Zero;
        if (originRef != 0 && TryGetPoint(originRef, out var p))
        {
            origin = new Vector2(p.X, p.Y);
        }

        transform = new Transform2D(
            Origin: origin,
            AxisX: axisX,
            AxisY: axisY,
            ScaleX: (float)scaleX,
            ScaleY: (float)scaleY);

        return true;
    }

    private bool TryGetCurve2D(uint curveRef, out List<Vector2> points)
    {
        if (_curve2DCache.TryGetValue(curveRef, out var cached))
        {
            points = new List<Vector2>(cached);
            return true;
        }

        points = new List<Vector2>();
        var curveType = _loader.GetLineType(curveRef);

        if (curveType == _ifcPolylineType || curveType == _ifcPolyLoopType)
        {
            var pointRefs = ReadRefSetArg(curveRef, 0);
            foreach (var pointRef in pointRefs)
            {
                if (!TryGetPoint(pointRef, out var point))
                {
                    continue;
                }

                points.Add(new Vector2(point.X, point.Y));
            }
        }
        else if (curveType == _ifcIndexedPolyCurveType)
        {
            var pointListRef = ReadOptionalRefArg(curveRef, 0);
            if (pointListRef != 0 && TryGetPointList2D(pointListRef, out var indexedPoints))
            {
                points.AddRange(indexedPoints);
            }
        }
        else if (curveType == _ifcCompositeCurveType)
        {
            var segmentRefs = ReadRefSetArg(curveRef, 0);
            foreach (var segmentRef in segmentRefs)
            {
                if (!TryGetCurveSegment2D(segmentRef, out var segment))
                {
                    continue;
                }

                AppendCurvePoints(points, segment);
            }
        }
        else if (curveType == _ifcTrimmedCurveType)
        {
            if (!TryGetTrimmedCurve2D(curveRef, out points))
            {
                return false;
            }
        }
        else if (curveType == _ifcCircleType || curveType == _ifcEllipseType)
        {
            if (!TryGetCircleLikeCurve2D(curveRef, curveType == _ifcEllipseType, out points))
            {
                return false;
            }
        }
        else if (curveType == _ifcBsplineCurveType || curveType == _ifcBsplineCurveWithKnotsType)
        {
            if (!TryGetBsplineCurve2D(curveRef, curveType == _ifcBsplineCurveWithKnotsType, out points))
            {
                return false;
            }
        }

        if (points.Count > 1 && NearlyEqual(points[0], points[^1]))
        {
            points.RemoveAt(points.Count - 1);
        }

        if (points.Count < 3)
        {
            return false;
        }

        _curve2DCache[curveRef] = new List<Vector2>(points);
        return true;
    }

    private bool TryGetCurve3D(uint curveRef, out List<Vector3> points)
    {
        if (_curve3DCache.TryGetValue(curveRef, out var cached))
        {
            points = new List<Vector3>(cached);
            return true;
        }

        points = new List<Vector3>();
        var curveType = _loader.GetLineType(curveRef);

        if (curveType == _ifcPolylineType || curveType == _ifcPolyLoopType)
        {
            var pointRefs = ReadRefSetArg(curveRef, 0);
            foreach (var pointRef in pointRefs)
            {
                if (TryGetPoint(pointRef, out var point))
                {
                    points.Add(point);
                }
            }
        }
        else if (curveType == _ifcIndexedPolyCurveType)
        {
            var pointListRef = ReadOptionalRefArg(curveRef, 0);
            if (pointListRef != 0 && TryGetPointList3D(pointListRef, out var pointList))
            {
                points.AddRange(pointList);
            }
        }
        else if (curveType == _ifcCompositeCurveType)
        {
            var segmentRefs = ReadRefSetArg(curveRef, 0);
            foreach (var segmentRef in segmentRefs)
            {
                if (!TryGetCurveSegment3D(segmentRef, out var segment))
                {
                    continue;
                }

                AppendCurvePoints(points, segment);
            }
        }
        else if (curveType == _ifcTrimmedCurveType)
        {
            if (!TryGetTrimmedCurve3D(curveRef, out points))
            {
                return false;
            }
        }
        else if (curveType == _ifcCircleType || curveType == _ifcEllipseType)
        {
            if (!TryGetCircleLikeCurve3D(curveRef, curveType == _ifcEllipseType, out points))
            {
                return false;
            }
        }
        else if (curveType == _ifcBsplineCurveType || curveType == _ifcBsplineCurveWithKnotsType)
        {
            if (!TryGetBsplineCurve3D(curveRef, curveType == _ifcBsplineCurveWithKnotsType, out points))
            {
                return false;
            }
        }

        if (points.Count > 1 && Vector3.DistanceSquared(points[0], points[^1]) < 1e-8f)
        {
            points.RemoveAt(points.Count - 1);
        }

        if (points.Count < 2)
        {
            return false;
        }

        _curve3DCache[curveRef] = new List<Vector3>(points);
        return true;
    }

    private bool IsCurveClosedByDefinition(uint curveRef)
    {
        var curveType = _loader.GetLineType(curveRef);
        if (curveType == _ifcPolylineType || curveType == _ifcPolyLoopType)
        {
            var pointRefs = ReadRefSetArg(curveRef, 0);
            if (pointRefs.Count <= 2)
            {
                return false;
            }

            if (pointRefs[0] == pointRefs[^1])
            {
                return true;
            }

            if (TryGetPoint(pointRefs[0], out var firstPoint) && TryGetPoint(pointRefs[^1], out var lastPoint))
            {
                return Vector3.DistanceSquared(firstPoint, lastPoint) < 1e-8f;
            }

            return false;
        }

        return false;
    }

    private bool TryGetCurveSegment2D(uint segmentRef, out List<Vector2> points)
    {
        points = new List<Vector2>();
        var type = _loader.GetLineType(segmentRef);
        if (type == _ifcCompositeCurveSegmentType)
        {
            var sameSense = ReadLogicalArg(segmentRef, 1, defaultValue: true);
            var parentRef = ReadOptionalRefArg(segmentRef, 2);
            if (parentRef == 0 || !TryGetCurve2D(parentRef, out points))
            {
                return false;
            }

            if (!sameSense)
            {
                points.Reverse();
            }

            return points.Count >= 2;
        }

        return TryGetCurve2D(segmentRef, out points);
    }

    private bool TryGetCurveSegment3D(uint segmentRef, out List<Vector3> points)
    {
        points = new List<Vector3>();
        var type = _loader.GetLineType(segmentRef);
        if (type == _ifcCompositeCurveSegmentType)
        {
            var sameSense = ReadLogicalArg(segmentRef, 1, defaultValue: true);
            var parentRef = ReadOptionalRefArg(segmentRef, 2);
            if (parentRef == 0 || !TryGetCurve3D(parentRef, out points))
            {
                return false;
            }

            if (!sameSense)
            {
                points.Reverse();
            }

            return points.Count >= 2;
        }

        return TryGetCurve3D(segmentRef, out points);
    }

    private bool TryGetTrimmedCurve2D(uint curveRef, out List<Vector2> points)
    {
        points = new List<Vector2>();
        var basisCurveRef = ReadOptionalRefArg(curveRef, 0);
        if (basisCurveRef == 0)
        {
            return false;
        }

        var basisType = _loader.GetLineType(basisCurveRef);
        if (basisType == _ifcCircleType || basisType == _ifcEllipseType)
        {
            if (!TryGetTrimmedCircleLikeCurve2D(curveRef, basisCurveRef, basisType == _ifcEllipseType, out points))
            {
                return false;
            }

            return points.Count >= 2;
        }

        if (!TryGetCurve2D(basisCurveRef, out points))
        {
            return false;
        }

        if (!ReadLogicalArg(curveRef, 3, defaultValue: true))
        {
            points.Reverse();
        }

        return points.Count >= 3;
    }

    private bool TryGetTrimmedCurve3D(uint curveRef, out List<Vector3> points)
    {
        points = new List<Vector3>();
        var basisCurveRef = ReadOptionalRefArg(curveRef, 0);
        if (basisCurveRef == 0)
        {
            return false;
        }

        var basisType = _loader.GetLineType(basisCurveRef);
        if (basisType == _ifcCircleType || basisType == _ifcEllipseType)
        {
            if (!TryGetTrimmedCircleLikeCurve3D(curveRef, basisCurveRef, basisType == _ifcEllipseType, out points))
            {
                return false;
            }

            return points.Count >= 2;
        }

        if (!TryGetCurve3D(basisCurveRef, out points))
        {
            return false;
        }

        if (!ReadLogicalArg(curveRef, 3, defaultValue: true))
        {
            points.Reverse();
        }

        return points.Count >= 2;
    }

    private bool TryGetTrimmedCircleLikeCurve2D(uint curveRef, uint basisCurveRef, bool isEllipse, out List<Vector2> points)
    {
        points = new List<Vector2>();

        if (!TryGetCircleLikeDefinition2D(basisCurveRef, isEllipse, out var placement, out var radiusX, out var radiusY))
        {
            return false;
        }

        double? ResolvePointRef(uint refId)
        {
            if (!TryGetPoint(refId, out var point))
            {
                return null;
            }

            var p2 = new Vector2(point.X, point.Y);
            var rel = p2 - placement.Origin;
            var localX = Vector2.Dot(rel, placement.AxisX);
            var localY = Vector2.Dot(rel, placement.AxisY);
            return TryComputeEllipseAngleDegrees(localX, localY, radiusX, radiusY, out var angle) ? angle : null;
        }

        if (!TryReadTrimAngle(curveRef, 1, ResolvePointRef, out var startAngleDeg) ||
            !TryReadTrimAngle(curveRef, 2, ResolvePointRef, out var endAngleDeg))
        {
            return TryGetCircleLikeCurve2D(basisCurveRef, isEllipse, out points);
        }

        var senseAgreement = ReadLogicalArg(curveRef, 3, defaultValue: true);
        var sweepDeg = ComputeTrimSweepDegrees(startAngleDeg, endAngleDeg, senseAgreement);
        var segments = DefaultCircleSegments + 2;

        for (var i = 0; i <= segments; i++)
        {
            var t = i / (double)segments;
            var angleDeg = senseAgreement
                ? startAngleDeg + sweepDeg * t
                : startAngleDeg - sweepDeg * t;
            var angle = angleDeg * (Math.PI / 180.0);
            var local = new Vector2((float)(radiusX * Math.Cos(angle)), (float)(radiusY * Math.Sin(angle)));
            points.Add(placement.TransformPoint(local));
        }

        return points.Count >= 2;
    }

    private bool TryGetTrimmedCircleLikeCurve3D(uint curveRef, uint basisCurveRef, bool isEllipse, out List<Vector3> points)
    {
        points = new List<Vector3>();

        if (!TryGetCircleLikeDefinition3D(
                basisCurveRef,
                isEllipse,
                out var hasPlacement3D,
                out var axis3,
                out var hasPlacement2D,
                out var axis2,
                out var radiusX,
                out var radiusY))
        {
            return false;
        }

        double? ResolvePointRef(uint refId)
        {
            if (!TryGetPoint(refId, out var point))
            {
                return null;
            }

            double localX;
            double localY;

            if (hasPlacement3D)
            {
                var relative = point - axis3.Origin;
                localX = Vector3.Dot(relative, axis3.AxisX);
                localY = Vector3.Dot(relative, axis3.AxisY);
            }
            else if (hasPlacement2D)
            {
                var p2 = new Vector2(point.X, point.Y);
                var rel = p2 - axis2.Origin;
                localX = Vector2.Dot(rel, axis2.AxisX);
                localY = Vector2.Dot(rel, axis2.AxisY);
            }
            else
            {
                localX = point.X;
                localY = point.Y;
            }

            return TryComputeEllipseAngleDegrees(localX, localY, radiusX, radiusY, out var angle) ? angle : null;
        }

        if (!TryReadTrimAngle(curveRef, 1, ResolvePointRef, out var startAngleDeg) ||
            !TryReadTrimAngle(curveRef, 2, ResolvePointRef, out var endAngleDeg))
        {
            return TryGetCircleLikeCurve3D(basisCurveRef, isEllipse, out points);
        }

        var senseAgreement = ReadLogicalArg(curveRef, 3, defaultValue: true);
        var sweepDeg = ComputeTrimSweepDegrees(startAngleDeg, endAngleDeg, senseAgreement);
        var segments = DefaultCircleSegments + 2;

        for (var i = 0; i <= segments; i++)
        {
            var t = i / (double)segments;
            var angleDeg = senseAgreement
                ? startAngleDeg + sweepDeg * t
                : startAngleDeg - sweepDeg * t;
            var angle = angleDeg * (Math.PI / 180.0);
            var local = new Vector3((float)(radiusX * Math.Cos(angle)), (float)(radiusY * Math.Sin(angle)), 0);
            if (hasPlacement3D)
            {
                points.Add(axis3.TransformPoint(local));
            }
            else if (hasPlacement2D)
            {
                var p2 = axis2.TransformPoint(new Vector2(local.X, local.Y));
                points.Add(new Vector3(p2, 0));
            }
            else
            {
                points.Add(local);
            }
        }

        return points.Count >= 2;
    }

    private bool TryGetCircleLikeCurve2D(uint curveRef, bool isEllipse, out List<Vector2> points)
    {
        points = new List<Vector2>();

        if (!TryGetCircleLikeDefinition2D(curveRef, isEllipse, out var placement, out var radiusX, out var radiusY))
        {
            return false;
        }

        for (var i = 0; i < DefaultCircleSegments; i++)
        {
            var angle = (2.0 * Math.PI * i) / DefaultCircleSegments;
            var local = new Vector2((float)(radiusX * Math.Cos(angle)), (float)(radiusY * Math.Sin(angle)));
            points.Add(placement.TransformPoint(local));
        }

        return points.Count >= 3;
    }

    private bool TryGetCircleLikeCurve3D(uint curveRef, bool isEllipse, out List<Vector3> points)
    {
        points = new List<Vector3>();

        if (!TryGetCircleLikeDefinition3D(
                curveRef,
                isEllipse,
                out var hasPlacement3D,
                out var axis3,
                out var hasPlacement2D,
                out var axis2,
                out var radiusX,
                out var radiusY))
        {
            return false;
        }

        for (var i = 0; i < DefaultCircleSegments; i++)
        {
            var angle = (2.0 * Math.PI * i) / DefaultCircleSegments;
            var local = new Vector3((float)(radiusX * Math.Cos(angle)), (float)(radiusY * Math.Sin(angle)), 0);
            if (hasPlacement3D)
            {
                points.Add(axis3.TransformPoint(local));
            }
            else if (hasPlacement2D)
            {
                var p2 = axis2.TransformPoint(new Vector2(local.X, local.Y));
                points.Add(new Vector3(p2, 0));
            }
            else
            {
                points.Add(local);
            }
        }

        return points.Count >= 3;
    }

    private bool TryGetCircleLikeDefinition2D(
        uint curveRef,
        bool isEllipse,
        out Placement2D placement,
        out double radiusX,
        out double radiusY)
    {
        placement = Placement2D.Identity;
        radiusX = ReadNumericArg(curveRef, 1, defaultValue: 0);
        if (radiusX <= 1e-8)
        {
            radiusY = 0;
            return false;
        }

        radiusY = isEllipse ? ReadNumericArg(curveRef, 2, defaultValue: radiusX) : radiusX;
        radiusY = Math.Max(radiusY, 1e-8);

        var placementRef = ReadOptionalRefArg(curveRef, 0);
        if (placementRef != 0 && TryGetAxis2Placement2D(placementRef, out var axis2))
        {
            placement = axis2;
        }

        return true;
    }

    private bool TryGetCircleLikeDefinition3D(
        uint curveRef,
        bool isEllipse,
        out bool hasPlacement3D,
        out Placement3D axis3,
        out bool hasPlacement2D,
        out Placement2D axis2,
        out double radiusX,
        out double radiusY)
    {
        axis3 = Placement3D.Identity;
        axis2 = Placement2D.Identity;
        hasPlacement3D = false;
        hasPlacement2D = false;

        radiusX = ReadNumericArg(curveRef, 1, defaultValue: 0);
        if (radiusX <= 1e-8)
        {
            radiusY = 0;
            return false;
        }

        radiusY = isEllipse ? ReadNumericArg(curveRef, 2, defaultValue: radiusX) : radiusX;
        radiusY = Math.Max(radiusY, 1e-8);

        var placementRef = ReadOptionalRefArg(curveRef, 0);
        if (placementRef != 0)
        {
            hasPlacement3D = TryGetAxis2Placement3D(placementRef, out axis3);
            hasPlacement2D = TryGetAxis2Placement2D(placementRef, out axis2);
        }

        return true;
    }

    private bool TryReadTrimAngle(uint curveRef, uint trimArgIndex, Func<uint, double?> resolvePointRef, out double angleDegrees)
    {
        angleDegrees = 0;

        _loader.MoveToLineArgument(curveRef, trimArgIndex);
        var token = _loader.GetTokenType();
        _loader.StepBack();
        if (token != IfcTokenType.SetBegin)
        {
            return false;
        }

        var trims = _loader.GetSetArgument();
        foreach (var trim in trims)
        {
            var trimToken = _loader.GetTokenType(trim);
            if ((trimToken == IfcTokenType.Real || trimToken == IfcTokenType.Integer) &&
                TryReadNumericAtOffset(trim, out var numeric))
            {
                angleDegrees = NormalizeAngleDegrees(ConvertAngleToDegrees(numeric));
                return true;
            }

            if (trimToken == IfcTokenType.Ref)
            {
                var refId = _loader.GetRefArgument(trim);
                var resolved = resolvePointRef(refId);
                if (resolved.HasValue)
                {
                    angleDegrees = NormalizeAngleDegrees(resolved.Value);
                    return true;
                }
            }
        }

        return false;
    }

    private static bool TryComputeEllipseAngleDegrees(double localX, double localY, double radiusX, double radiusY, out double angleDegrees)
    {
        angleDegrees = 0;
        if (Math.Abs(radiusX) < 1e-8 || Math.Abs(radiusY) < 1e-8)
        {
            return false;
        }

        var nx = localX / radiusX;
        var ny = localY / radiusY;
        if (nx * nx + ny * ny < 1e-12)
        {
            return false;
        }

        angleDegrees = NormalizeAngleDegrees(Math.Atan2(ny, nx) * (180.0 / Math.PI));
        return true;
    }

    private static double ComputeTrimSweepDegrees(double startAngleDegrees, double endAngleDegrees, bool senseAgreement)
    {
        var start = NormalizeAngleDegrees(startAngleDegrees);
        var end = NormalizeAngleDegrees(endAngleDegrees);

        var sweep = senseAgreement ? end - start : start - end;
        if (sweep <= 1e-9)
        {
            sweep += 360.0;
        }

        return Math.Min(sweep, 360.0);
    }

    private static double NormalizeAngleDegrees(double angleDegrees)
    {
        var normalized = angleDegrees % 360.0;
        if (normalized < 0)
        {
            normalized += 360.0;
        }

        return normalized;
    }

    private bool TryGetBsplineCurve2D(uint curveRef, bool hasExplicitKnots, out List<Vector2> points)
    {
        points = new List<Vector2>();
        var degree = (int)Math.Round(ReadNumericArg(curveRef, 0, defaultValue: 3));
        degree = Math.Max(1, degree);

        var controlPointRefs = ReadRefSetArg(curveRef, 1);
        var controls = new List<Vector3>(controlPointRefs.Count);
        foreach (var controlPointRef in controlPointRefs)
        {
            if (TryGetPoint(controlPointRef, out var controlPoint))
            {
                controls.Add(controlPoint);
            }
        }

        if (controls.Count < degree + 1)
        {
            return false;
        }

        var knots = hasExplicitKnots && TryReadExpandedKnots(curveRef, 5, 6, out var expandedKnots)
            ? expandedKnots
            : BuildUniformOpenKnotVector(degree, controls.Count);

        if (knots.Count != controls.Count + degree + 1)
        {
            knots = BuildUniformOpenKnotVector(degree, controls.Count);
        }

        var samples = EvaluateBsplineCurve(controls, degree, knots, sampleCount: Math.Max(controls.Count * 8, 24));
        foreach (var sample in samples)
        {
            points.Add(new Vector2(sample.X, sample.Y));
        }

        return points.Count >= 3;
    }

    private bool TryGetBsplineCurve3D(uint curveRef, bool hasExplicitKnots, out List<Vector3> points)
    {
        points = new List<Vector3>();
        var degree = (int)Math.Round(ReadNumericArg(curveRef, 0, defaultValue: 3));
        degree = Math.Max(1, degree);

        var controlPointRefs = ReadRefSetArg(curveRef, 1);
        var controls = new List<Vector3>(controlPointRefs.Count);
        foreach (var controlPointRef in controlPointRefs)
        {
            if (TryGetPoint(controlPointRef, out var controlPoint))
            {
                controls.Add(controlPoint);
            }
        }

        if (controls.Count < degree + 1)
        {
            return false;
        }

        var knots = hasExplicitKnots && TryReadExpandedKnots(curveRef, 5, 6, out var expandedKnots)
            ? expandedKnots
            : BuildUniformOpenKnotVector(degree, controls.Count);

        if (knots.Count != controls.Count + degree + 1)
        {
            knots = BuildUniformOpenKnotVector(degree, controls.Count);
        }

        points = EvaluateBsplineCurve(controls, degree, knots, sampleCount: Math.Max(controls.Count * 8, 24));
        return points.Count >= 2;
    }

    private bool TryReadExpandedKnots(uint expressId, uint multiplicityArg, uint knotArg, out List<double> knots)
    {
        knots = new List<double>();

        _loader.MoveToLineArgument(expressId, multiplicityArg);
        var multiplicityToken = _loader.GetTokenType();
        _loader.StepBack();
        if (multiplicityToken != IfcTokenType.SetBegin)
        {
            return false;
        }

        _loader.MoveToLineArgument(expressId, knotArg);
        var knotToken = _loader.GetTokenType();
        _loader.StepBack();
        if (knotToken != IfcTokenType.SetBegin)
        {
            return false;
        }

        _loader.MoveToLineArgument(expressId, multiplicityArg);
        var multiplicityOffsets = _loader.GetSetArgument();
        _loader.MoveToLineArgument(expressId, knotArg);
        var knotOffsets = _loader.GetSetArgument();

        if (multiplicityOffsets.Count == 0 || knotOffsets.Count == 0 || multiplicityOffsets.Count != knotOffsets.Count)
        {
            return false;
        }

        for (var i = 0; i < multiplicityOffsets.Count; i++)
        {
            var mOffset = multiplicityOffsets[i];
            var kOffset = knotOffsets[i];

            if (_loader.GetTokenType(mOffset) != IfcTokenType.Integer ||
                !TryReadNumericAtOffset(kOffset, out var knotValue))
            {
                return false;
            }

            var multiplicity = _loader.GetIntArgument(mOffset);
            if (multiplicity <= 0)
            {
                continue;
            }

            for (var j = 0; j < multiplicity; j++)
            {
                knots.Add(knotValue);
            }
        }

        return knots.Count > 0;
    }

    private bool TryGetControlGrid3D(uint expressId, uint argumentIndex, out List<List<Vector3>> controlGrid)
    {
        controlGrid = new List<List<Vector3>>();

        _loader.MoveToLineArgument(expressId, argumentIndex);
        var tk = _loader.GetTokenType();
        _loader.StepBack();
        if (tk != IfcTokenType.SetBegin)
        {
            return false;
        }

        var setList = _loader.GetSetListArgument();
        foreach (var rowOffsets in setList)
        {
            var row = new List<Vector3>(rowOffsets.Count);
            foreach (var offset in rowOffsets)
            {
                if (_loader.GetTokenType(offset) != IfcTokenType.Ref)
                {
                    continue;
                }

                var pointRef = _loader.GetRefArgument(offset);
                if (TryGetPoint(pointRef, out var point))
                {
                    row.Add(point);
                }
            }

            if (row.Count > 0)
            {
                controlGrid.Add(row);
            }
        }

        if (controlGrid.Count == 0)
        {
            return false;
        }

        var expectedCols = controlGrid[0].Count;
        return expectedCols > 0 && controlGrid.All(r => r.Count == expectedCols);
    }

    private bool TryGetWeightGrid(uint expressId, uint argumentIndex, int expectedRows, int expectedCols, out List<List<double>> weights)
    {
        weights = new List<List<double>>();

        _loader.MoveToLineArgument(expressId, argumentIndex);
        var tk = _loader.GetTokenType();
        _loader.StepBack();
        if (tk != IfcTokenType.SetBegin)
        {
            return false;
        }

        var setList = _loader.GetSetListArgument();
        foreach (var rowOffsets in setList)
        {
            var row = new List<double>(rowOffsets.Count);
            foreach (var offset in rowOffsets)
            {
                if (TryReadNumericAtOffset(offset, out var weight))
                {
                    row.Add(weight);
                }
            }

            if (row.Count > 0)
            {
                weights.Add(row);
            }
        }

        if (weights.Count != expectedRows)
        {
            return false;
        }

        return weights.All(r => r.Count == expectedCols);
    }

    private bool TryGetBsplineSurfaceDefinition(uint surfaceRef, out BsplineSurfaceDefinition definition)
    {
        definition = default;
        var surfaceType = _loader.GetLineType(surfaceRef);
        var hasKnots = surfaceType == _ifcBsplineSurfaceWithKnotsType || surfaceType == _ifcRationalBsplineSurfaceWithKnotsType;
        var hasWeights = surfaceType == _ifcRationalBsplineSurfaceWithKnotsType;

        if (surfaceType != _ifcBsplineSurfaceType
            && surfaceType != _ifcBsplineSurfaceWithKnotsType
            && surfaceType != _ifcRationalBsplineSurfaceWithKnotsType)
        {
            return false;
        }

        var degreeU = (int)Math.Round(ReadNumericArg(surfaceRef, 0, defaultValue: 3));
        var degreeV = (int)Math.Round(ReadNumericArg(surfaceRef, 1, defaultValue: 3));
        degreeU = Math.Max(1, degreeU);
        degreeV = Math.Max(1, degreeV);

        if (!TryGetControlGrid3D(surfaceRef, 2, out var controls))
        {
            return false;
        }

        var rows = controls.Count;
        var cols = controls[0].Count;
        if (rows < degreeU + 1 || cols < degreeV + 1)
        {
            return false;
        }

        var weights = new List<List<double>>(rows);
        for (var r = 0; r < rows; r++)
        {
            var row = new List<double>(cols);
            for (var c = 0; c < cols; c++)
            {
                row.Add(1.0);
            }

            weights.Add(row);
        }

        if (hasWeights && !TryGetWeightGrid(surfaceRef, 12, rows, cols, out weights))
        {
            return false;
        }

        var knotsU = hasKnots && TryReadExpandedKnots(surfaceRef, 7, 9, out var ku)
            ? ku
            : BuildUniformOpenKnotVector(degreeU, rows);
        var knotsV = hasKnots && TryReadExpandedKnots(surfaceRef, 8, 10, out var kv)
            ? kv
            : BuildUniformOpenKnotVector(degreeV, cols);

        if (knotsU.Count != rows + degreeU + 1)
        {
            knotsU = BuildUniformOpenKnotVector(degreeU, rows);
        }

        if (knotsV.Count != cols + degreeV + 1)
        {
            knotsV = BuildUniformOpenKnotVector(degreeV, cols);
        }

        var rangeU = new Vector2((float)knotsU[degreeU], (float)knotsU[rows]);
        var rangeV = new Vector2((float)knotsV[degreeV], (float)knotsV[cols]);

        if (Math.Abs(rangeU.Y - rangeU.X) < 1e-10f)
        {
            rangeU = new Vector2(rangeU.X, rangeU.X + 1);
        }

        if (Math.Abs(rangeV.Y - rangeV.X) < 1e-10f)
        {
            rangeV = new Vector2(rangeV.X, rangeV.X + 1);
        }

        definition = new BsplineSurfaceDefinition(
            DegreeU: degreeU,
            DegreeV: degreeV,
            ControlPoints: controls,
            Weights: weights,
            KnotsU: knotsU,
            KnotsV: knotsV,
            RangeU: rangeU,
            RangeV: rangeV);
        return true;
    }

    private static Vector2 InverseEvaluateNurbsSurfacePoint(BsplineSurfaceDefinition surface, Vector3 target)
    {
        var minError = 0.0001;
        var maxError = 0.01;
        var fU = (surface.RangeU.X + surface.RangeU.Y) * 0.5;
        var fV = (surface.RangeV.X + surface.RangeV.Y) * 0.5;

        var bestDistance = float.MaxValue;
        var divisor = 100.0;

        while (bestDistance > maxError && divisor < 10_000.0)
        {
            for (var r = 1; r < 5; r++)
            {
                var mulDivisor = r * r * divisor;
                for (var round = 0; round < 3 && bestDistance > minError; round++)
                {
                    for (var i = 0; i < 6; i++)
                    {
                        var angle = (i / 6.0) * (Math.PI * 2.0);
                        var incU = Math.Sin(angle) / mulDivisor;
                        var incV = Math.Cos(angle) / mulDivisor;

                        while (true)
                        {
                            var testU = WrapRange(fU + incU, surface.RangeU.X, surface.RangeU.Y);
                            var testV = WrapRange(fV + incV, surface.RangeV.X, surface.RangeV.Y);
                            var point = EvaluateNurbsSurfacePoint(
                                surface.ControlPoints,
                                surface.Weights,
                                surface.DegreeU,
                                surface.DegreeV,
                                surface.KnotsU,
                                surface.KnotsV,
                                testU,
                                testV);

                            var distance = Vector3.Distance(point, target);
                            if (distance + 1e-12 < bestDistance)
                            {
                                bestDistance = distance;
                                fU = testU;
                                fV = testV;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
            }

            divisor *= 3.0;
        }

        return new Vector2((float)fU, (float)fV);
    }

    private static bool IsInsideTrimmedDomain(Vector2 uv, IReadOnlyList<Vector2> outer, IReadOnlyList<List<Vector2>> holes)
    {
        if (!IsPointInPolygon2D(uv, outer))
        {
            return false;
        }

        for (var i = 0; i < holes.Count; i++)
        {
            if (IsPointInPolygon2D(uv, holes[i]))
            {
                return false;
            }
        }

        return true;
    }

    private static bool IsPointInPolygon2D(Vector2 point, IReadOnlyList<Vector2> polygon)
    {
        if (polygon.Count < 3)
        {
            return false;
        }

        var inside = false;
        var j = polygon.Count - 1;
        for (var i = 0; i < polygon.Count; i++)
        {
            var pi = polygon[i];
            var pj = polygon[j];
            var intersects = ((pi.Y > point.Y) != (pj.Y > point.Y))
                             && (point.X < (pj.X - pi.X) * (point.Y - pi.Y) / ((pj.Y - pi.Y) + 1e-20f) + pi.X);
            if (intersects)
            {
                inside = !inside;
            }

            j = i;
        }

        return inside;
    }

    private static double WrapRange(double value, double min, double max)
    {
        var range = max - min;
        if (Math.Abs(range) < 1e-12)
        {
            return min;
        }

        while (value < min)
        {
            value += range;
        }

        while (value > max)
        {
            value -= range;
        }

        return value;
    }

    private static Vector3 EvaluateNurbsSurfacePoint(
        IReadOnlyList<List<Vector3>> controls,
        IReadOnlyList<List<double>> weights,
        int degreeU,
        int degreeV,
        IReadOnlyList<double> knotsU,
        IReadOnlyList<double> knotsV,
        double u,
        double v)
    {
        var rows = controls.Count;
        var cols = controls[0].Count;

        var sum = Vector3.Zero;
        double weightSum = 0;

        for (var i = 0; i < rows; i++)
        {
            var nu = BasisFunction(i, degreeU, u, knotsU);
            if (Math.Abs(nu) < 1e-14)
            {
                continue;
            }

            for (var j = 0; j < cols; j++)
            {
                var nv = BasisFunction(j, degreeV, v, knotsV);
                if (Math.Abs(nv) < 1e-14)
                {
                    continue;
                }

                var w = weights[i][j];
                var basis = nu * nv * w;
                if (Math.Abs(basis) < 1e-14)
                {
                    continue;
                }

                sum += controls[i][j] * (float)basis;
                weightSum += basis;
            }
        }

        if (Math.Abs(weightSum) < 1e-12)
        {
            return controls[0][0];
        }

        return sum / (float)weightSum;
    }

    private static double BasisFunction(int i, int degree, double u, IReadOnlyList<double> knots)
    {
        if (degree == 0)
        {
            var left = knots[i];
            var right = knots[i + 1];
            var isLast = i + 1 == knots.Count - 1;
            return (u >= left && u < right) || (isLast && Math.Abs(u - right) < 1e-12) ? 1.0 : 0.0;
        }

        var denom1 = knots[i + degree] - knots[i];
        var denom2 = knots[i + degree + 1] - knots[i + 1];

        var term1 = 0.0;
        var term2 = 0.0;

        if (Math.Abs(denom1) > 1e-14)
        {
            term1 = (u - knots[i]) / denom1 * BasisFunction(i, degree - 1, u, knots);
        }

        if (Math.Abs(denom2) > 1e-14)
        {
            term2 = (knots[i + degree + 1] - u) / denom2 * BasisFunction(i + 1, degree - 1, u, knots);
        }

        return term1 + term2;
    }

    private static List<double> BuildUniformOpenKnotVector(int degree, int controlCount)
    {
        var knotCount = controlCount + degree + 1;
        var knots = new List<double>(knotCount);

        var interiorDenominator = Math.Max(1, controlCount - degree);
        for (var i = 0; i < knotCount; i++)
        {
            if (i <= degree)
            {
                knots.Add(0);
            }
            else if (i >= controlCount)
            {
                knots.Add(1);
            }
            else
            {
                knots.Add((double)(i - degree) / interiorDenominator);
            }
        }

        return knots;
    }

    private static List<Vector3> EvaluateBsplineCurve(IReadOnlyList<Vector3> controls, int degree, IReadOnlyList<double> knots, int sampleCount)
    {
        var result = new List<Vector3>(sampleCount);
        if (controls.Count < degree + 1 || knots.Count < controls.Count + degree + 1)
        {
            return result;
        }

        sampleCount = Math.Max(sampleCount, 2);
        var start = knots[degree];
        var end = knots[controls.Count];
        if (Math.Abs(end - start) < 1e-10)
        {
            end = start + 1;
        }

        for (var i = 0; i < sampleCount; i++)
        {
            var t = i / (double)(sampleCount - 1);
            var u = start + (end - start) * t;
            result.Add(DeBoor(degree, knots, controls, u));
        }

        return result;
    }

    private static Vector3 DeBoor(int degree, IReadOnlyList<double> knots, IReadOnlyList<Vector3> controls, double u)
    {
        var n = controls.Count - 1;
        var span = FindKnotSpan(n, degree, u, knots);
        var d = new Vector3[degree + 1];

        for (var j = 0; j <= degree; j++)
        {
            d[j] = controls[span - degree + j];
        }

        for (var r = 1; r <= degree; r++)
        {
            for (var j = degree; j >= r; j--)
            {
                var i = span - degree + j;
                var denom = knots[i + degree - r + 1] - knots[i];
                var alpha = Math.Abs(denom) < 1e-12 ? 0 : (u - knots[i]) / denom;
                d[j] = Vector3.Lerp(d[j - 1], d[j], (float)alpha);
            }
        }

        return d[degree];
    }

    private static int FindKnotSpan(int n, int degree, double u, IReadOnlyList<double> knots)
    {
        if (u >= knots[n + 1])
        {
            return n;
        }

        if (u <= knots[degree])
        {
            return degree;
        }

        var low = degree;
        var high = n + 1;
        var mid = (low + high) / 2;

        while (u < knots[mid] || u >= knots[mid + 1])
        {
            if (u < knots[mid])
            {
                high = mid;
            }
            else
            {
                low = mid;
            }

            mid = (low + high) / 2;
        }

        return mid;
    }

    private static void AppendCurvePoints(List<Vector2> target, IReadOnlyList<Vector2> segment)
    {
        for (var i = 0; i < segment.Count; i++)
        {
            var point = segment[i];
            if (target.Count > 0 && NearlyEqual(target[^1], point))
            {
                continue;
            }

            target.Add(point);
        }
    }

    private static void AppendCurvePoints(List<Vector3> target, IReadOnlyList<Vector3> segment)
    {
        for (var i = 0; i < segment.Count; i++)
        {
            var point = segment[i];
            if (target.Count > 0 && Vector3.DistanceSquared(target[^1], point) < 1e-8f)
            {
                continue;
            }

            target.Add(point);
        }
    }

    private bool TryGetPointList2D(uint pointListRef, out List<Vector2> points)
    {
        points = new List<Vector2>();
        var type = _loader.GetLineType(pointListRef);
        if (type != _ifcCartesianPointList2DType && type != _ifcCartesianPointList3DType)
        {
            return false;
        }

        _loader.MoveToLineArgument(pointListRef, 0);
        var tk = _loader.GetTokenType();
        _loader.StepBack();
        if (tk != IfcTokenType.SetBegin)
        {
            return false;
        }

        var setList = _loader.GetSetListArgument();
        foreach (var pointOffsets in setList)
        {
            if (pointOffsets.Count < 2)
            {
                continue;
            }

            if (!TryReadNumericAtOffset(pointOffsets[0], out var x) ||
                !TryReadNumericAtOffset(pointOffsets[1], out var y))
            {
                continue;
            }

            points.Add(new Vector2((float)x, (float)y));
        }

        return points.Count >= 3;
    }

    private bool TryGetPointList3D(uint pointListRef, out List<Vector3> points)
    {
        points = new List<Vector3>();
        var type = _loader.GetLineType(pointListRef);
        if (type != _ifcCartesianPointList2DType && type != _ifcCartesianPointList3DType)
        {
            return false;
        }

        _loader.MoveToLineArgument(pointListRef, 0);
        var tk = _loader.GetTokenType();
        _loader.StepBack();
        if (tk != IfcTokenType.SetBegin)
        {
            return false;
        }

        var setList = _loader.GetSetListArgument();
        foreach (var pointOffsets in setList)
        {
            if (pointOffsets.Count < 2)
            {
                continue;
            }

            if (!TryReadNumericAtOffset(pointOffsets[0], out var x) ||
                !TryReadNumericAtOffset(pointOffsets[1], out var y))
            {
                continue;
            }

            var z = 0d;
            if (pointOffsets.Count > 2)
            {
                _ = TryReadNumericAtOffset(pointOffsets[2], out z);
            }

            points.Add(new Vector3((float)x, (float)y, (float)z));
        }

        return points.Count >= 2;
    }

    private bool TryGetPoint(uint pointRef, out Vector3 point)
    {
        if (_pointCache.TryGetValue(pointRef, out point))
        {
            return true;
        }

        point = Vector3.Zero;
        if (_loader.GetLineType(pointRef) != _ifcCartesianPointType)
        {
            return false;
        }

        _loader.MoveToLineArgument(pointRef, 0);
        var tk = _loader.GetTokenType();
        _loader.StepBack();
        if (tk != IfcTokenType.SetBegin)
        {
            return false;
        }

        var offsets = _loader.GetSetArgument();
        if (offsets.Count < 2)
        {
            return false;
        }

        if (!TryReadNumericAtOffset(offsets[0], out var x) ||
            !TryReadNumericAtOffset(offsets[1], out var y))
        {
            return false;
        }

        var z = 0d;
        if (offsets.Count > 2)
        {
            _ = TryReadNumericAtOffset(offsets[2], out z);
        }

        point = new Vector3((float)x, (float)y, (float)z);
        _pointCache[pointRef] = point;
        return true;
    }

    private bool TryGetDirection(uint directionRef, out Vector3 direction)
    {
        if (_directionCache.TryGetValue(directionRef, out direction))
        {
            return true;
        }

        direction = Vector3.UnitZ;
        if (_loader.GetLineType(directionRef) != _ifcDirectionType)
        {
            return false;
        }

        _loader.MoveToLineArgument(directionRef, 0);
        var tk = _loader.GetTokenType();
        _loader.StepBack();
        if (tk != IfcTokenType.SetBegin)
        {
            return false;
        }

        var offsets = _loader.GetSetArgument();
        if (offsets.Count < 2)
        {
            return false;
        }

        if (!TryReadNumericAtOffset(offsets[0], out var x) ||
            !TryReadNumericAtOffset(offsets[1], out var y))
        {
            return false;
        }

        var z = 0d;
        if (offsets.Count > 2)
        {
            _ = TryReadNumericAtOffset(offsets[2], out z);
        }

        var dir = new Vector3((float)x, (float)y, (float)z);
        direction = SafeNormalize(dir, Vector3.UnitZ);
        _directionCache[directionRef] = direction;
        return true;
    }

    private bool TryGetAxis2Placement3D(uint placementRef, out Placement3D placement)
    {
        if (_axis3Cache.TryGetValue(placementRef, out placement))
        {
            return true;
        }

        placement = Placement3D.Identity;
        if (_loader.GetLineType(placementRef) != _ifcAxis2Placement3DType)
        {
            return false;
        }

        var locationRef = ReadOptionalRefArg(placementRef, 0);
        var axisRef = ReadOptionalRefArg(placementRef, 1);
        var refDirectionRef = ReadOptionalRefArg(placementRef, 2);

        var origin = locationRef != 0 && TryGetPoint(locationRef, out var locationPoint)
            ? locationPoint
            : Vector3.Zero;

        var axisZ = axisRef != 0 && TryGetDirection(axisRef, out var axisDirection)
            ? axisDirection
            : Vector3.UnitZ;
        axisZ = SafeNormalize(axisZ, Vector3.UnitZ);

        var axisXSeed = refDirectionRef != 0 && TryGetDirection(refDirectionRef, out var refDirection)
            ? refDirection
            : Vector3.UnitX;
        axisXSeed = SafeNormalize(axisXSeed, Vector3.UnitX);

        var axisY = Vector3.Cross(axisZ, axisXSeed);
        if (axisY.LengthSquared() < 1e-8f)
        {
            axisY = Vector3.Cross(axisZ, Math.Abs(axisZ.X) > 0.99f ? Vector3.UnitY : Vector3.UnitX);
        }

        axisY = SafeNormalize(axisY, Vector3.UnitY);
        var axisX = SafeNormalize(Vector3.Cross(axisY, axisZ), Vector3.UnitX);

        placement = new Placement3D(origin, axisX, axisY, axisZ);
        _axis3Cache[placementRef] = placement;
        return true;
    }

    private bool TryGetAxis1Placement(uint placementRef, out Axis1Placement placement)
    {
        if (_axis1Cache.TryGetValue(placementRef, out placement))
        {
            return true;
        }

        placement = Axis1Placement.Identity;
        if (_loader.GetLineType(placementRef) != _ifcAxis1PlacementType)
        {
            return false;
        }

        var locationRef = ReadOptionalRefArg(placementRef, 0);
        if (locationRef == 0 || !TryGetPoint(locationRef, out var origin))
        {
            return false;
        }

        var axisRef = ReadOptionalRefArg(placementRef, 1);
        var axis = axisRef != 0 && TryGetDirection(axisRef, out var axisDirection)
            ? axisDirection
            : Vector3.UnitZ;
        axis = SafeNormalize(axis, Vector3.UnitZ);

        var xAxis = Vector3.UnitX;
        if (Math.Abs(Vector3.Dot(xAxis, axis)) > 0.9f)
        {
            xAxis = Vector3.UnitY;
        }

        var yAxis = SafeNormalize(Vector3.Cross(axis, xAxis), Vector3.UnitY);
        xAxis = SafeNormalize(Vector3.Cross(yAxis, axis), Vector3.UnitX);

        placement = new Axis1Placement(origin, axis, xAxis, yAxis);
        _axis1Cache[placementRef] = placement;
        return true;
    }

    private bool TryGetAxis2Placement2D(uint placementRef, out Placement2D placement)
    {
        if (_axis2Cache.TryGetValue(placementRef, out placement))
        {
            return true;
        }

        placement = Placement2D.Identity;
        if (_loader.GetLineType(placementRef) != _ifcAxis2Placement2DType)
        {
            return false;
        }

        var locationRef = ReadOptionalRefArg(placementRef, 0);
        var directionRef = ReadOptionalRefArg(placementRef, 1);

        Vector2 origin2;
        if (locationRef != 0 && TryGetPoint(locationRef, out var locationPoint))
        {
            origin2 = new Vector2(locationPoint.X, locationPoint.Y);
        }
        else
        {
            origin2 = Vector2.Zero;
        }

        Vector2 axisX;
        if (directionRef != 0 && TryGetDirection(directionRef, out var direction3))
        {
            var dir2 = new Vector2(direction3.X, direction3.Y);
            axisX = SafeNormalize(dir2, Vector2.UnitX);
        }
        else
        {
            axisX = Vector2.UnitX;
        }

        var axisY = new Vector2(-axisX.Y, axisX.X);
        placement = new Placement2D(origin2, axisX, axisY);
        _axis2Cache[placementRef] = placement;
        return true;
    }

    private uint[] BuildExtrusionIndices(int vertexCount, IReadOnlyList<Int3> capTriangles)
    {
        if (vertexCount < 3)
        {
            return Array.Empty<uint>();
        }

        var indices = new List<uint>(capTriangles.Count * 6 + vertexCount * 6);

        foreach (var tri in capTriangles)
        {
            // Bottom cap (reversed orientation).
            indices.Add((uint)tri.A);
            indices.Add((uint)tri.C);
            indices.Add((uint)tri.B);

            // Top cap.
            indices.Add((uint)(vertexCount + tri.A));
            indices.Add((uint)(vertexCount + tri.B));
            indices.Add((uint)(vertexCount + tri.C));
        }

        for (var i = 0; i < vertexCount; i++)
        {
            var j = (i + 1) % vertexCount;
            uint bi = (uint)i;
            uint bj = (uint)j;
            uint ti = (uint)(vertexCount + i);
            uint tj = (uint)(vertexCount + j);

            indices.Add(bi);
            indices.Add(bj);
            indices.Add(tj);

            indices.Add(bi);
            indices.Add(tj);
            indices.Add(ti);
        }

        return indices.ToArray();
    }

    private static List<Vector2> NormalizePolygon(IReadOnlyList<Vector2> raw)
    {
        var result = new List<Vector2>(raw.Count);
        for (var i = 0; i < raw.Count; i++)
        {
            var p = raw[i];
            if (result.Count > 0 && NearlyEqual(result[^1], p))
            {
                continue;
            }

            result.Add(p);
        }

        if (result.Count > 1 && NearlyEqual(result[0], result[^1]))
        {
            result.RemoveAt(result.Count - 1);
        }

        return result;
    }

    private static List<Int3> TriangulatePolygon(IReadOnlyList<Vector2> polygon)
    {
        if (polygon.Count < 3)
        {
            return new List<Int3>();
        }

        var working = new List<int>(polygon.Count);
        for (var i = 0; i < polygon.Count; i++)
        {
            working.Add(i);
        }

        var triangles = new List<Int3>(Math.Max(1, polygon.Count - 2));
        var ccw = SignedArea(polygon) >= 0;
        var guard = 0;
        var maxGuard = polygon.Count * polygon.Count;

        while (working.Count > 3 && guard < maxGuard)
        {
            guard++;
            var earFound = false;

            for (var i = 0; i < working.Count; i++)
            {
                var prev = working[(i - 1 + working.Count) % working.Count];
                var curr = working[i];
                var next = working[(i + 1) % working.Count];

                if (!IsConvex(polygon[prev], polygon[curr], polygon[next], ccw))
                {
                    continue;
                }

                var containsPoint = false;
                for (var j = 0; j < working.Count; j++)
                {
                    var candidate = working[j];
                    if (candidate == prev || candidate == curr || candidate == next)
                    {
                        continue;
                    }

                    if (PointInTriangle(polygon[candidate], polygon[prev], polygon[curr], polygon[next]))
                    {
                        containsPoint = true;
                        break;
                    }
                }

                if (containsPoint)
                {
                    continue;
                }

                triangles.Add(new Int3(prev, curr, next));
                working.RemoveAt(i);
                earFound = true;
                break;
            }

            if (!earFound)
            {
                break;
            }
        }

        if (working.Count == 3)
        {
            triangles.Add(new Int3(working[0], working[1], working[2]));
        }

        if (!ccw)
        {
            for (var i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                triangles[i] = new Int3(tri.A, tri.C, tri.B);
            }
        }

        return triangles;
    }

    private static float SignedArea(IReadOnlyList<Vector2> polygon)
    {
        var area = 0f;
        for (var i = 0; i < polygon.Count; i++)
        {
            var j = (i + 1) % polygon.Count;
            area += polygon[i].X * polygon[j].Y - polygon[j].X * polygon[i].Y;
        }

        return area * 0.5f;
    }

    private static bool IsConvex(Vector2 prev, Vector2 curr, Vector2 next, bool ccw)
    {
        var cross = Cross(next - curr, prev - curr);
        return ccw ? cross > 1e-7f : cross < -1e-7f;
    }

    private static bool PointInTriangle(Vector2 point, Vector2 a, Vector2 b, Vector2 c)
    {
        var ab = Sign(point, a, b);
        var bc = Sign(point, b, c);
        var ca = Sign(point, c, a);

        var hasNeg = ab < 0 || bc < 0 || ca < 0;
        var hasPos = ab > 0 || bc > 0 || ca > 0;
        return !(hasNeg && hasPos);
    }

    private static float Sign(Vector2 p1, Vector2 p2, Vector2 p3)
    {
        return (p1.X - p3.X) * (p2.Y - p3.Y) - (p2.X - p3.X) * (p1.Y - p3.Y);
    }

    private static float Cross(Vector2 a, Vector2 b)
    {
        return a.X * b.Y - a.Y * b.X;
    }

    private static bool NearlyEqual(Vector2 a, Vector2 b)
    {
        return Vector2.DistanceSquared(a, b) < 1e-8f;
    }

    private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
    {
        if (value.LengthSquared() < 1e-8f)
        {
            return fallback;
        }

        return Vector3.Normalize(value);
    }

    private static Vector2 SafeNormalize(Vector2 value, Vector2 fallback)
    {
        if (value.LengthSquared() < 1e-8f)
        {
            return fallback;
        }

        return Vector2.Normalize(value);
    }

    private static IfcParsedColor CreateColor(uint expressId)
    {
        var hue = (expressId % 360) / 360f;
        var saturation = 0.35f;
        var value = 0.82f;
        return HsvToRgb(hue, saturation, value);
    }

    private static IfcParsedColor HsvToRgb(float h, float s, float v)
    {
        var i = (int)MathF.Floor(h * 6f);
        var f = h * 6f - i;
        var p = v * (1f - s);
        var q = v * (1f - f * s);
        var t = v * (1f - (1f - f) * s);

        var mod = i % 6;
        return mod switch
        {
            0 => new IfcParsedColor(v, t, p, 1f),
            1 => new IfcParsedColor(q, v, p, 1f),
            2 => new IfcParsedColor(p, v, t, 1f),
            3 => new IfcParsedColor(p, q, v, 1f),
            4 => new IfcParsedColor(t, p, v, 1f),
            _ => new IfcParsedColor(v, p, q, 1f)
        };
    }

    private bool ReadLogicalArg(uint expressId, uint argumentIndex, bool defaultValue)
    {
        _loader.MoveToLineArgument(expressId, argumentIndex);
        var tk = _loader.GetTokenType();
        _loader.StepBack();

        if (tk is not (IfcTokenType.Enum or IfcTokenType.Label or IfcTokenType.String))
        {
            return defaultValue;
        }

        var value = _loader.GetStringArgument();
        if (string.IsNullOrWhiteSpace(value))
        {
            return defaultValue;
        }

        value = value.Trim().Trim('.').ToUpperInvariant();
        return value switch
        {
            "T" or "TRUE" => true,
            "F" or "FALSE" => false,
            _ => defaultValue
        };
    }

    private uint ReadOptionalRefArg(uint expressId, uint argumentIndex)
    {
        _loader.MoveToLineArgument(expressId, argumentIndex);
        var tk = _loader.GetTokenType();
        _loader.StepBack();

        if (tk == IfcTokenType.Ref)
        {
            return _loader.GetRefArgument();
        }

        return 0;
    }

    private double ReadNumericArg(uint expressId, uint argumentIndex, double defaultValue)
    {
        _loader.MoveToLineArgument(expressId, argumentIndex);
        var tk = _loader.GetTokenType();
        _loader.StepBack();

        return tk switch
        {
            IfcTokenType.Real => _loader.GetDoubleArgument(),
            IfcTokenType.Integer => _loader.GetIntArgument(),
            _ => defaultValue
        };
    }

    private List<uint> ReadRefSetArg(uint expressId, uint argumentIndex)
    {
        _loader.MoveToLineArgument(expressId, argumentIndex);
        var tk = _loader.GetTokenType();
        _loader.StepBack();
        if (tk != IfcTokenType.SetBegin)
        {
            return new List<uint>();
        }

        var refs = new List<uint>();
        var offsets = _loader.GetSetArgument();
        foreach (var offset in offsets)
        {
            if (_loader.GetTokenType(offset) != IfcTokenType.Ref)
            {
                continue;
            }

            refs.Add(_loader.GetRefArgument(offset));
        }

        return refs;
    }

    private bool TryReadNumericAtOffset(uint offset, out double value)
    {
        value = 0;
        var tk = _loader.GetTokenType(offset);
        switch (tk)
        {
            case IfcTokenType.Real:
                value = _loader.GetDoubleArgument(offset);
                return true;
            case IfcTokenType.Integer:
                value = _loader.GetIntArgument(offset);
                return true;
            default:
                return false;
        }
    }

    private static double ConvertAngleToDegrees(double angleValue)
    {
        // IFC often uses radians; support both radians and degrees input defensively.
        if (Math.Abs(angleValue) <= Math.PI * 2.0 + 1e-6)
        {
            return angleValue * (180.0 / Math.PI);
        }

        return angleValue;
    }

    private static void ApplyPlacementToGeometry(Geometry geometry, Placement3D placement)
    {
        for (var i = 0u; i < geometry.NumPoints; i++)
        {
            var p = geometry.GetPoint(i);
            var wp = placement.TransformPoint(p);
            geometry.SetPoint(wp.X, wp.Y, wp.Z, i);
        }
    }

    private static IfcParsedMesh BuildParsedMeshFromGeometry(uint expressId, Geometry geometry, IfcParsedColor color)
    {
        var positions = new float[geometry.NumPoints * 3];
        for (var i = 0u; i < geometry.NumPoints; i++)
        {
            var p = geometry.GetPoint(i);
            var o = i * 3;
            positions[o] = p.X;
            positions[o + 1] = p.Y;
            positions[o + 2] = p.Z;
        }

        var indices = geometry.IndexData.ToArray();
        return new IfcParsedMesh(expressId, positions, indices, color);
    }

    private static Geometry ToOperationsGeometry(IfcParsedMesh mesh)
    {
        var geometry = new Geometry();
        var indices = mesh.Indices;
        var positions = mesh.Positions;

        for (var i = 0; i + 2 < indices.Length; i += 3)
        {
            var ia = (int)indices[i] * 3;
            var ib = (int)indices[i + 1] * 3;
            var ic = (int)indices[i + 2] * 3;

            if (ia + 2 >= positions.Length || ib + 2 >= positions.Length || ic + 2 >= positions.Length)
            {
                continue;
            }

            var a = new Vector3(positions[ia], positions[ia + 1], positions[ia + 2]);
            var b = new Vector3(positions[ib], positions[ib + 1], positions[ib + 2]);
            var c = new Vector3(positions[ic], positions[ic + 1], positions[ic + 2]);
            geometry.AddFace(a, b, c);
        }

        geometry.BuildPlanes();
        return geometry;
    }

    private readonly record struct Int3(int A, int B, int C);

    private readonly record struct BsplineSurfaceDefinition(
        int DegreeU,
        int DegreeV,
        List<List<Vector3>> ControlPoints,
        List<List<double>> Weights,
        List<double> KnotsU,
        List<double> KnotsV,
        Vector2 RangeU,
        Vector2 RangeV);

    private readonly record struct Placement3D(Vector3 Origin, Vector3 AxisX, Vector3 AxisY, Vector3 AxisZ)
    {
        public static Placement3D Identity => new(Vector3.Zero, Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ);

        public Vector3 TransformPoint(Vector3 localPoint)
        {
            return Origin + TransformDirection(localPoint);
        }

        public Vector3 TransformDirection(Vector3 localDirection)
        {
            return AxisX * localDirection.X + AxisY * localDirection.Y + AxisZ * localDirection.Z;
        }
    }

    private readonly record struct Placement2D(Vector2 Origin, Vector2 AxisX, Vector2 AxisY)
    {
        public static Placement2D Identity => new(Vector2.Zero, Vector2.UnitX, Vector2.UnitY);

        public Vector2 TransformPoint(Vector2 localPoint)
        {
            return Origin + AxisX * localPoint.X + AxisY * localPoint.Y;
        }
    }

    private readonly record struct Transform2D(Vector2 Origin, Vector2 AxisX, Vector2 AxisY, float ScaleX, float ScaleY)
    {
        public static Transform2D Identity => new(Vector2.Zero, Vector2.UnitX, Vector2.UnitY, 1f, 1f);

        public Vector2 TransformPoint(Vector2 localPoint)
        {
            var scaled = new Vector2(localPoint.X * ScaleX, localPoint.Y * ScaleY);
            return Origin + AxisX * scaled.X + AxisY * scaled.Y;
        }
    }

    private readonly record struct Axis1Placement(Vector3 Origin, Vector3 AxisZ, Vector3 AxisX, Vector3 AxisY)
    {
        public static Axis1Placement Identity => new(Vector3.Zero, Vector3.UnitZ, Vector3.UnitX, Vector3.UnitY);

        public Matrix4x4 ToMatrix()
        {
            return new Matrix4x4(
                AxisX.X, AxisX.Y, AxisX.Z, 0,
                AxisY.X, AxisY.Y, AxisY.Z, 0,
                AxisZ.X, AxisZ.Y, AxisZ.Z, 0,
                Origin.X, Origin.Y, Origin.Z, 1
            );
        }
    }
}
