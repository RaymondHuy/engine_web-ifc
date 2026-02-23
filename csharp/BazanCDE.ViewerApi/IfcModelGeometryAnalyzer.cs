using System.Numerics;
using BazanCDE.Parsing;

internal sealed class IfcModelGeometryAnalyzer
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
    private readonly IfcLoader _loader;

    private readonly uint _ifcCartesianPointType;
    private readonly uint _ifcDirectionType;
    private readonly uint _ifcAxis2Placement3DType;
    private readonly uint _ifcAxis2Placement2DType;
    private readonly uint _ifcPolylineType;
    private readonly uint _ifcPolyLoopType;
    private readonly uint _ifcIndexedPolyCurveType;
    private readonly uint _ifcCartesianPointList2DType;
    private readonly uint _ifcCartesianPointList3DType;
    private readonly uint _ifcArbitraryClosedProfileDefType;
    private readonly uint _ifcArbitraryProfileDefWithVoidsType;
    private readonly uint _ifcRectangleProfileDefType;
    private readonly uint _ifcExtrudedAreaSolidType;

    private readonly Dictionary<uint, Vector3> _pointCache = new();
    private readonly Dictionary<uint, Vector3> _directionCache = new();
    private readonly Dictionary<uint, Placement3D> _axis3Cache = new();
    private readonly Dictionary<uint, Placement2D> _axis2Cache = new();
    private readonly Dictionary<uint, List<Vector2>> _curve2DCache = new();

    public IfcExtrudedGeometryExtractor(IfcLoader loader, DefaultIfcSchemaManager schemaManager)
    {
        _loader = loader;

        _ifcCartesianPointType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANPOINT");
        _ifcDirectionType = schemaManager.IfcTypeToTypeCode("IFCDIRECTION");
        _ifcAxis2Placement3DType = schemaManager.IfcTypeToTypeCode("IFCAXIS2PLACEMENT3D");
        _ifcAxis2Placement2DType = schemaManager.IfcTypeToTypeCode("IFCAXIS2PLACEMENT2D");
        _ifcPolylineType = schemaManager.IfcTypeToTypeCode("IFCPOLYLINE");
        _ifcPolyLoopType = schemaManager.IfcTypeToTypeCode("IFCPOLYLOOP");
        _ifcIndexedPolyCurveType = schemaManager.IfcTypeToTypeCode("IFCINDEXEDPOLYCURVE");
        _ifcCartesianPointList2DType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANPOINTLIST2D");
        _ifcCartesianPointList3DType = schemaManager.IfcTypeToTypeCode("IFCCARTESIANPOINTLIST3D");
        _ifcArbitraryClosedProfileDefType = schemaManager.IfcTypeToTypeCode("IFCARBITRARYCLOSEDPROFILEDEF");
        _ifcArbitraryProfileDefWithVoidsType = schemaManager.IfcTypeToTypeCode("IFCARBITRARYPROFILEDEFWITHVOIDS");
        _ifcRectangleProfileDefType = schemaManager.IfcTypeToTypeCode("IFCRECTANGLEPROFILEDEF");
        _ifcExtrudedAreaSolidType = schemaManager.IfcTypeToTypeCode("IFCEXTRUDEDAREASOLID");
    }

    public IfcParsedModel Extract(int maxMeshes, int maxTriangles)
    {
        maxMeshes = Math.Max(1, maxMeshes);
        maxTriangles = Math.Max(1, maxTriangles);

        var meshes = new List<IfcParsedMesh>(Math.Min(maxMeshes, 512));
        var triangleCount = 0;

        foreach (var expressId in _loader.GetAllLines())
        {
            if (_loader.GetLineType(expressId) != _ifcExtrudedAreaSolidType)
            {
                continue;
            }

            if (meshes.Count >= maxMeshes || triangleCount >= maxTriangles)
            {
                break;
            }

            if (!TryBuildExtrudedMesh(expressId, out var mesh, out var trianglesInMesh))
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

    private bool TryBuildExtrudedMesh(uint expressId, out IfcParsedMesh mesh, out int triangleCount)
    {
        mesh = default!;
        triangleCount = 0;

        var profileRef = ReadOptionalRefArg(expressId, 0);
        if (profileRef == 0 || !TryGetProfileOutline2D(profileRef, out var outline))
        {
            return false;
        }

        var polygon = NormalizePolygon(outline);
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

    private bool TryGetProfileOutline2D(uint profileRef, out List<Vector2> points)
    {
        points = new List<Vector2>();
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

            points = rectangle;
            return true;
        }

        if (type == _ifcArbitraryClosedProfileDefType || type == _ifcArbitraryProfileDefWithVoidsType)
        {
            // IFCARBITRARYPROFILEDEFWITHVOIDS can have inner loops; this extractor currently uses outer loop only.
            var curveRef = ReadOptionalRefArg(profileRef, 2);
            return curveRef != 0 && TryGetCurve2D(curveRef, out points);
        }

        return false;
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
        return ccw ? cross < -1e-7f : cross > 1e-7f;
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

    private readonly record struct Int3(int A, int B, int C);

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
}

internal sealed record IfcParsedModel(
    int MeshCount,
    int TriangleCount,
    IReadOnlyList<IfcParsedMesh> Meshes
);

internal sealed record IfcParsedMesh(
    uint ExpressId,
    float[] Positions,
    uint[] Indices,
    IfcParsedColor Color
);

internal sealed record IfcParsedColor(float R, float G, float B, float A);
