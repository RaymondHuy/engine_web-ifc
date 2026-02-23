using System.Numerics;
using System.Globalization;

namespace BazanCDE.Parsing.Operations;

internal static class BooleanEps
{
    public const double TolerancePlaneDeviation = 1.0E-04;
    public const double ToleranceBackDeviationDistance = 1.0E-04;
    public const double ToleranceInsideOutsidePerimeter = 1.0E-10;
    public const double ToleranceParallel = 1.0E-04;
    public const double ToleranceParallelTight = 1.0E-10;
}

internal enum MeshLocation
{
    Inside,
    Outside,
    Boundary
}

internal readonly record struct InsideResult(MeshLocation Location, Vector3 Normal);

internal sealed class BoolBvhNode
{
    public int Start { get; set; }
    public int End { get; set; }
    public int Left { get; set; } = -1;
    public int Right { get; set; } = -1;
    public Aabb Box { get; } = new();

    public bool IsLeaf => Left < 0 && Right < 0;
}

internal sealed class BoolBvh
{
    private readonly List<Aabb> _boxes;
    private readonly List<BoolBvhNode> _nodes = new();

    public BoolBvh(Geometry geometry)
    {
        Geometry = geometry;
        _boxes = new List<Aabb>((int)geometry.NumFaces);
        for (var i = 0u; i < geometry.NumFaces; i++)
        {
            _boxes.Add(geometry.GetFaceBox(i));
        }

        Box = new Aabb();
        for (var i = 0; i < _boxes.Count; i++)
        {
            Box.Merge(_boxes[i]);
        }

        if (_boxes.Count > 0)
        {
            BuildNode(0, _boxes.Count, axis: 1, depth: 0);
        }
    }

    public Geometry Geometry { get; }

    public Aabb Box { get; }

    public bool IntersectRay(Vector3 origin, Vector3 direction, Func<uint, bool> callback)
    {
        if (_nodes.Count == 0)
        {
            return false;
        }

        var stack = new Stack<int>();
        stack.Push(0);

        while (stack.Count > 0)
        {
            var node = _nodes[stack.Pop()];
            if (!node.Box.IntersectRay(origin, direction))
            {
                continue;
            }

            if (node.IsLeaf)
            {
                for (var i = node.Start; i < node.End; i++)
                {
                    var box = _boxes[i];
                    if (!box.IntersectRay(origin, direction))
                    {
                        continue;
                    }

                    if (callback(box.Index))
                    {
                        return true;
                    }
                }
            }
            else
            {
                stack.Push(node.Left);
                stack.Push(node.Right);
            }
        }

        return false;
    }

    private int BuildNode(int start, int end, int axis, int depth)
    {
        var node = new BoolBvhNode
        {
            Start = start,
            End = end
        };

        var nodeId = _nodes.Count;
        _nodes.Add(node);

        for (var i = start; i < end; i++)
        {
            node.Box.Merge(_boxes[i]);
        }

        var size = end - start;
        if (depth >= 6 || size <= 12)
        {
            return nodeId;
        }

        var middle = (start + end) / 2;
        _boxes.Sort(
            start,
            size,
            Comparer<Aabb>.Create(
                (first, second) => GetAxis(first.Center, axis).CompareTo(GetAxis(second.Center, axis))));

        node.Left = BuildNode(start, middle, (axis + 1) % 3, depth + 1);
        node.Right = BuildNode(middle, end, (axis + 1) % 3, depth + 1);
        return nodeId;
    }

    private static float GetAxis(Vector3 value, int axis)
    {
        return axis switch
        {
            0 => value.X,
            1 => value.Y,
            _ => value.Z
        };
    }
}

internal static class RayTriangleIntersector
{
    public static bool IntersectRayTriangle(
        Vector3 origin,
        Vector3 direction,
        Vector3 v0,
        Vector3 v1,
        Vector3 v2,
        out Vector3 hitPosition,
        out double t,
        out double dPlane)
    {
        hitPosition = Vector3.Zero;
        t = 0;
        dPlane = 0;

        var v0v1 = v1 - v0;
        var v0v2 = v2 - v0;
        var normal = Vector3.Cross(v0v1, v0v2);

        var nDotRayDirection = Vector3.Dot(normal, direction);
        if (Math.Abs(nDotRayDirection) < BooleanEps.ToleranceParallelTight)
        {
            return false;
        }

        var d = -Vector3.Dot(normal, v0);
        var dOrigin = Vector3.Dot(normal, origin);

        t = -(dOrigin + d) / nDotRayDirection;
        if (t < -BooleanEps.ToleranceBackDeviationDistance)
        {
            return false;
        }

        var p = origin + direction * (float)t;
        var normalLength = normal.Length();
        if (normalLength < Epsilons.EpsNonZero)
        {
            return false;
        }

        var normalUnit = normal / normalLength;
        dPlane = Vector3.Dot(direction * (float)t, normalUnit);

        var edge0 = v1 - v0;
        var v0p = p - v0;
        var c = Vector3.Cross(edge0, v0p);
        if (Vector3.Dot(normal, c) < -BooleanEps.ToleranceInsideOutsidePerimeter)
        {
            return false;
        }

        var edge1 = v2 - v1;
        var v1p = p - v1;
        c = Vector3.Cross(edge1, v1p);
        if (Vector3.Dot(normal, c) < -BooleanEps.ToleranceInsideOutsidePerimeter)
        {
            return false;
        }

        var edge2 = v0 - v2;
        var v2p = p - v2;
        c = Vector3.Cross(edge2, v2p);
        if (Vector3.Dot(normal, c) < -BooleanEps.ToleranceInsideOutsidePerimeter)
        {
            return false;
        }

        hitPosition = p;
        return true;
    }
}

internal static class MeshClassifier
{
    public static InsideResult IsInsideMesh(
        Vector3 point,
        Vector3 normal,
        Geometry geometry,
        BoolBvh bvh,
        Vector3? direction = null,
        bool unionMode = false)
    {
        var winding = 0;
        var rayDir = direction ?? new Vector3(1f, 1.1f, 1.4f);
        rayDir = SafeNormalize(rayDir + new Vector3(0.02f, 0.01f, 0.04f), Vector3.UnitX);

        var result = new InsideResult(MeshLocation.Boundary, Vector3.Zero);
        var hasBoundaryResult = bvh.IntersectRay(point, rayDir, faceIndex =>
        {
            var f = geometry.GetFace(faceIndex);
            var a = geometry.GetPoint((uint)f.I0);
            var b = geometry.GetPoint((uint)f.I1);
            var c = geometry.GetPoint((uint)f.I2);

            if (!RayTriangleIntersector.IntersectRayTriangle(
                    point,
                    rayDir,
                    a,
                    b,
                    c,
                    out _,
                    out _,
                    out var dPlane))
            {
                return false;
            }

            var otherNormal = SafeNormalize(Vector3.Cross(b - a, c - a), Vector3.UnitZ);
            var dn = Vector3.Dot(otherNormal, normal);
            if (Math.Abs(dPlane) < BooleanEps.TolerancePlaneDeviation)
            {
                if (dn > 1f - (float)BooleanEps.ToleranceParallel)
                {
                    result = new InsideResult(MeshLocation.Boundary, normal);
                    return true;
                }

                if (dn < -1f + (float)BooleanEps.ToleranceParallel)
                {
                    result = unionMode
                        ? new InsideResult(MeshLocation.Boundary, normal)
                        : new InsideResult(MeshLocation.Outside, normal);
                    return true;
                }

                result = new InsideResult(MeshLocation.Boundary, otherNormal);
                return true;
            }

            winding++;
            return false;
        });

        if (hasBoundaryResult)
        {
            return result;
        }

        return new InsideResult(
            winding % 2 == 1 ? MeshLocation.Inside : MeshLocation.Outside,
            Vector3.Zero);
    }

    private static Vector3 SafeNormalize(Vector3 v, Vector3 fallback)
    {
        if (v.LengthSquared() < 1e-12f)
        {
            return fallback;
        }

        return Vector3.Normalize(v);
    }
}

internal static class BooleanCsg
{
    public static Geometry Subtract(Geometry first, Geometry second)
    {
        var normalized = NormalizeRelevant(first, second, unionMode: false);
        var core = SubtractCore(normalized.RelevantA, normalized.RelevantB);

        var final = new Geometry();
        final.AddGeometry(core);
        final.AddGeometry(normalized.SharedPlaneMesh);
        AddFacesByIndices(final, first, normalized.IrrelevantATest);
        AddFacesByIndices(final, second, normalized.IrrelevantBTest);
        AddFacesByIndices(final, first, normalized.IrrelevantA);
        return final;
    }

    public static Geometry Union(Geometry first, Geometry second)
    {
        var normalized = NormalizeRelevant(first, second, unionMode: true);
        var core = UnionCore(normalized.RelevantA, normalized.RelevantB);

        var final = new Geometry();
        final.AddGeometry(core);
        final.AddGeometry(normalized.SharedPlaneMesh);
        AddFacesByIndices(final, first, normalized.IrrelevantATest);
        AddFacesByIndices(final, second, normalized.IrrelevantBTest);
        AddFacesByIndices(final, first, normalized.IrrelevantA);
        AddFacesByIndices(final, second, normalized.IrrelevantB);
        return final;
    }

    public static Geometry Intersect(Geometry first, Geometry second)
    {
        var normalized = NormalizeRelevant(first, second, unionMode: false);
        var core = IntersectCore(normalized.RelevantA, normalized.RelevantB);

        var final = new Geometry();
        final.AddGeometry(core);
        final.AddGeometry(normalized.SharedPlaneMesh);
        return final;
    }

    private static Geometry SubtractCore(Geometry first, Geometry second)
    {
        var result = new Geometry();
        if (first.NumFaces == 0)
        {
            return result;
        }

        if (second.NumFaces == 0)
        {
            result.AddGeometry(first);
            return result;
        }

        var bvhFirst = new BoolBvh(first);
        var bvhSecond = new BoolBvh(second);
        var seen = new HashSet<string>(StringComparer.Ordinal);

        for (var i = 0u; i < first.NumFaces; i++)
        {
            var f = first.GetFace(i);
            var a = first.GetPoint((uint)f.I0);
            var b = first.GetPoint((uint)f.I1);
            var c = first.GetPoint((uint)f.I2);
            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                continue;
            }

            var center = (a + b * 1.02f + c * 1.03f) / 3.05f;
            var isInsideSecond = StableInsideCheck(center, n, second, bvhSecond, unionMode: false);

            if (isInsideSecond.Location == MeshLocation.Outside)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
            else if (isInsideSecond.Location == MeshLocation.Boundary && Vector3.Dot(n, isInsideSecond.Normal) < 0)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
        }

        for (var i = 0u; i < second.NumFaces; i++)
        {
            var f = second.GetFace(i);
            var a = second.GetPoint((uint)f.I0);
            var b = second.GetPoint((uint)f.I1);
            var c = second.GetPoint((uint)f.I2);
            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                continue;
            }

            var center = (a + b * 1.02f + c * 1.03f) / 3.05f;
            var isInsideFirst = StableInsideCheck(center, n, first, bvhFirst, unionMode: false);

            if (isInsideFirst.Location == MeshLocation.Inside)
            {
                AddUniqueFace(result, b, a, c, f.PlaneId, seen);
            }
            else if (isInsideFirst.Location == MeshLocation.Boundary)
            {
                if (Vector3.Dot(n, isInsideFirst.Normal) < 0)
                {
                    AddUniqueFace(result, a, b, c, f.PlaneId, seen);
                }
                else
                {
                    AddUniqueFace(result, b, a, c, f.PlaneId, seen);
                }
            }
        }

        return result;
    }

    private static Geometry UnionCore(Geometry first, Geometry second)
    {
        var result = new Geometry();
        if (first.NumFaces == 0)
        {
            result.AddGeometry(second);
            return result;
        }

        if (second.NumFaces == 0)
        {
            result.AddGeometry(first);
            return result;
        }

        var bvhFirst = new BoolBvh(first);
        var bvhSecond = new BoolBvh(second);
        var seen = new HashSet<string>(StringComparer.Ordinal);

        for (var i = 0u; i < first.NumFaces; i++)
        {
            var f = first.GetFace(i);
            var a = first.GetPoint((uint)f.I0);
            var b = first.GetPoint((uint)f.I1);
            var c = first.GetPoint((uint)f.I2);
            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                continue;
            }

            var center = (a + b * 1.02f + c * 1.03f) / 3.05f;
            var isInsideSecond = StableInsideCheck(center, n, second, bvhSecond, unionMode: true);

            if (isInsideSecond.Location == MeshLocation.Outside)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
            else if (isInsideSecond.Location == MeshLocation.Boundary && Vector3.Dot(n, isInsideSecond.Normal) < 0)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
        }

        for (var i = 0u; i < second.NumFaces; i++)
        {
            var f = second.GetFace(i);
            var a = second.GetPoint((uint)f.I0);
            var b = second.GetPoint((uint)f.I1);
            var c = second.GetPoint((uint)f.I2);
            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                continue;
            }

            var center = (a + b * 1.02f + c * 1.03f) / 3.05f;
            var isInsideFirst = StableInsideCheck(center, n, first, bvhFirst, unionMode: true);

            if (isInsideFirst.Location == MeshLocation.Outside)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
            else if (isInsideFirst.Location == MeshLocation.Boundary && Vector3.Dot(n, isInsideFirst.Normal) < 0)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
        }

        return result;
    }

    private static Geometry IntersectCore(Geometry first, Geometry second)
    {
        var result = new Geometry();
        if (first.NumFaces == 0 || second.NumFaces == 0)
        {
            return result;
        }

        var bvhFirst = new BoolBvh(first);
        var bvhSecond = new BoolBvh(second);
        var seen = new HashSet<string>(StringComparer.Ordinal);

        for (var i = 0u; i < first.NumFaces; i++)
        {
            var f = first.GetFace(i);
            var a = first.GetPoint((uint)f.I0);
            var b = first.GetPoint((uint)f.I1);
            var c = first.GetPoint((uint)f.I2);
            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                continue;
            }

            var center = (a + b * 1.02f + c * 1.03f) / 3.05f;
            var isInsideSecond = StableInsideCheck(center, n, second, bvhSecond, unionMode: false);

            if (isInsideSecond.Location == MeshLocation.Inside)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
            else if (isInsideSecond.Location == MeshLocation.Boundary && Vector3.Dot(n, isInsideSecond.Normal) > 0)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
        }

        for (var i = 0u; i < second.NumFaces; i++)
        {
            var f = second.GetFace(i);
            var a = second.GetPoint((uint)f.I0);
            var b = second.GetPoint((uint)f.I1);
            var c = second.GetPoint((uint)f.I2);
            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                continue;
            }

            var center = (a + b * 1.02f + c * 1.03f) / 3.05f;
            var isInsideFirst = StableInsideCheck(center, n, first, bvhFirst, unionMode: false);

            if (isInsideFirst.Location == MeshLocation.Inside)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
            else if (isInsideFirst.Location == MeshLocation.Boundary && Vector3.Dot(n, isInsideFirst.Normal) > 0)
            {
                AddUniqueFace(result, a, b, c, f.PlaneId, seen);
            }
        }

        return result;
    }

    private sealed class NormalizedGeometries
    {
        public Geometry RelevantA { get; } = new();
        public Geometry RelevantB { get; } = new();
        public Geometry SharedPlaneMesh { get; } = new();
        public List<uint> IrrelevantA { get; } = new();
        public List<uint> IrrelevantATest { get; } = new();
        public List<uint> IrrelevantB { get; } = new();
        public List<uint> IrrelevantBTest { get; } = new();
    }

    private static NormalizedGeometries NormalizeRelevant(Geometry first, Geometry second, bool unionMode)
    {
        var normalized = new NormalizedGeometries();
        if (first.NumFaces == 0 && second.NumFaces == 0)
        {
            return normalized;
        }

        var firstBox = first.GetAabb();
        var secondBox = second.GetAabb();

        var firstFaceBoxes = new Aabb[first.NumFaces];
        var secondFaceBoxes = new Aabb[second.NumFaces];

        for (var i = 0u; i < first.NumFaces; i++)
        {
            firstFaceBoxes[i] = first.GetFaceBox(i);
        }

        for (var i = 0u; i < second.NumFaces; i++)
        {
            secondFaceBoxes[i] = second.GetFaceBox(i);
        }

        for (var i = 0u; i < first.NumFaces; i++)
        {
            var faceBox = firstFaceBoxes[i];
            if (!faceBox.Intersects(secondBox))
            {
                normalized.IrrelevantA.Add(i);
                continue;
            }

            var contact = false;
            for (var j = 0u; j < second.NumFaces; j++)
            {
                if (faceBox.Intersects(secondFaceBoxes[j]))
                {
                    contact = true;
                    break;
                }
            }

            if (!contact)
            {
                normalized.IrrelevantATest.Add(i);
                continue;
            }

            AddFaceByIndex(normalized.RelevantA, first, i);
        }

        for (var i = 0u; i < second.NumFaces; i++)
        {
            var faceBox = secondFaceBoxes[i];
            if (!faceBox.Intersects(firstBox))
            {
                normalized.IrrelevantB.Add(i);
                continue;
            }

            var contact = false;
            for (var j = 0u; j < first.NumFaces; j++)
            {
                if (faceBox.Intersects(firstFaceBoxes[j]))
                {
                    contact = true;
                    break;
                }
            }

            if (!contact)
            {
                if (unionMode || second.NumFaces < 2000)
                {
                    normalized.IrrelevantBTest.Add(i);
                }

                continue;
            }

            AddFaceByIndex(normalized.RelevantB, second, i);
        }

        normalized.SharedPlaneMesh.AddGeometry(BuildSharedPlaneTriangulation(normalized.RelevantA, normalized.RelevantB));

        return normalized;
    }

    private static void AddFacesByIndices(Geometry target, Geometry source, IReadOnlyList<uint> faceIndices)
    {
        for (var i = 0; i < faceIndices.Count; i++)
        {
            AddFaceByIndex(target, source, faceIndices[i]);
        }
    }

    private static void AddFaceByIndex(Geometry target, Geometry source, uint faceIndex)
    {
        if (faceIndex >= source.NumFaces)
        {
            return;
        }

        var f = source.GetFace(faceIndex);
        target.AddFace(
            source.GetPoint((uint)f.I0),
            source.GetPoint((uint)f.I1),
            source.GetPoint((uint)f.I2),
            f.PlaneId);
    }

    private readonly record struct PlaneBucketKey(int Nx, int Ny, int Nz, int D);

    private readonly record struct Segment3(Vector3 A, Vector3 B, int Source);

    private readonly record struct Segment2(Vector2 A, Vector2 B, int Source);

    private sealed class PlaneBucket
    {
        public Vector3 Normal { get; init; }
        public double Distance { get; init; }
        public List<Segment3> Segments { get; } = new();
    }

    private sealed class EdgeAccumulator
    {
        public int Count { get; set; }
        public Vector3 A { get; set; }
        public Vector3 B { get; set; }
    }

    private readonly record struct PlaneBasis(Vector3 Origin, Vector3 AxisX, Vector3 AxisY)
    {
        public Vector2 Project(Vector3 point)
        {
            var relative = point - Origin;
            return new Vector2(Vector3.Dot(relative, AxisX), Vector3.Dot(relative, AxisY));
        }

        public Vector3 Unproject(Vector2 point)
        {
            return Origin + AxisX * point.X + AxisY * point.Y;
        }
    }

    private static Geometry BuildSharedPlaneTriangulation(Geometry first, Geometry second)
    {
        var result = new Geometry();
        if (first.NumFaces == 0 || second.NumFaces == 0)
        {
            return result;
        }

        var buckets = new Dictionary<PlaneBucketKey, PlaneBucket>();
        CollectBoundarySegments(first, source: 0, buckets);
        CollectBoundarySegments(second, source: 1, buckets);

        var bvhFirst = new BoolBvh(first);
        var bvhSecond = new BoolBvh(second);
        var seen = new HashSet<string>(StringComparer.Ordinal);

        foreach (var bucket in buckets.Values)
        {
            var hasA = false;
            var hasB = false;
            for (var i = 0; i < bucket.Segments.Count; i++)
            {
                if (bucket.Segments[i].Source == 0)
                {
                    hasA = true;
                }
                else
                {
                    hasB = true;
                }
            }

            if (!hasA || !hasB)
            {
                continue;
            }

            var basis = CreatePlaneBasis(bucket.Normal, bucket.Distance);
            var projected = new List<Segment2>(bucket.Segments.Count);
            for (var i = 0; i < bucket.Segments.Count; i++)
            {
                var segment = bucket.Segments[i];
                var pa = basis.Project(segment.A);
                var pb = basis.Project(segment.B);
                if (Vector2.DistanceSquared(pa, pb) < 1e-12f)
                {
                    continue;
                }

                projected.Add(new Segment2(pa, pb, segment.Source));
            }

            if (projected.Count < 3)
            {
                continue;
            }

            var splitSegments = SplitSegmentsAtIntersections(projected);
            var loops = BuildClosedLoops(splitSegments);
            for (var i = 0; i < loops.Count; i++)
            {
                var loop = loops[i];
                if (loop.Count < 3)
                {
                    continue;
                }

                var triangles = TriangulatePolygon2D(loop);
                for (var t = 0; t < triangles.Count; t++)
                {
                    var tri = triangles[t];
                    var p0 = basis.Unproject(loop[tri.A]);
                    var p1 = basis.Unproject(loop[tri.B]);
                    var p2 = basis.Unproject(loop[tri.C]);

                    var center = (p0 + p1 + p2) / 3f;
                    var inA = StableInsideCheck(center, bucket.Normal, first, bvhFirst, unionMode: true);
                    var inB = StableInsideCheck(center, bucket.Normal, second, bvhSecond, unionMode: true);

                    if (inA.Location == MeshLocation.Outside && inB.Location == MeshLocation.Outside)
                    {
                        continue;
                    }

                    if (GeometryOps.ComputeSafeNormal(p0, p1, p2, out var triNormal, Epsilons.EpsSmall)
                        && Vector3.Dot(triNormal, bucket.Normal) < 0)
                    {
                        (p1, p2) = (p2, p1);
                    }

                    AddUniqueFace(result, p0, p1, p2, planeId: -1, seen);
                }
            }
        }

        return result;
    }

    private static void CollectBoundarySegments(Geometry geometry, int source, Dictionary<PlaneBucketKey, PlaneBucket> buckets)
    {
        var edgesByPlane = new Dictionary<PlaneBucketKey, Dictionary<string, EdgeAccumulator>>();

        for (var faceIndex = 0u; faceIndex < geometry.NumFaces; faceIndex++)
        {
            var face = geometry.GetFace(faceIndex);
            var a = geometry.GetPoint((uint)face.I0);
            var b = geometry.GetPoint((uint)face.I1);
            var c = geometry.GetPoint((uint)face.I2);

            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var normal, Epsilons.EpsSmall))
            {
                continue;
            }

            var distance = Vector3.Dot(normal, a);
            var key = CreatePlaneKey(normal, distance, out var canonicalNormal, out var canonicalDistance);

            if (!edgesByPlane.TryGetValue(key, out var edgeMap))
            {
                edgeMap = new Dictionary<string, EdgeAccumulator>(StringComparer.Ordinal);
                edgesByPlane[key] = edgeMap;
            }

            AddOrCountEdge(edgeMap, a, b);
            AddOrCountEdge(edgeMap, b, c);
            AddOrCountEdge(edgeMap, c, a);

            if (!buckets.TryGetValue(key, out var bucket))
            {
                bucket = new PlaneBucket
                {
                    Normal = canonicalNormal,
                    Distance = canonicalDistance
                };
                buckets[key] = bucket;
            }
        }

        foreach (var (planeKey, edges) in edgesByPlane)
        {
            if (!buckets.TryGetValue(planeKey, out var bucket))
            {
                continue;
            }

            foreach (var edge in edges.Values)
            {
                if (edge.Count == 1 && Vector3.DistanceSquared(edge.A, edge.B) > 1e-12f)
                {
                    bucket.Segments.Add(new Segment3(edge.A, edge.B, source));
                }
            }
        }
    }

    private static void AddOrCountEdge(Dictionary<string, EdgeAccumulator> edgeMap, Vector3 a, Vector3 b)
    {
        var key = CreateUnorderedEdgeKey(a, b);
        if (!edgeMap.TryGetValue(key, out var accumulator))
        {
            accumulator = new EdgeAccumulator
            {
                Count = 1,
                A = a,
                B = b
            };
            edgeMap[key] = accumulator;
            return;
        }

        accumulator.Count++;
    }

    private static PlaneBucketKey CreatePlaneKey(Vector3 normal, double distance, out Vector3 canonicalNormal, out double canonicalDistance)
    {
        canonicalNormal = SafeNormalize(normal, Vector3.UnitZ);
        canonicalDistance = distance;

        var flip = canonicalNormal.Z < -1e-10f
            || (Math.Abs(canonicalNormal.Z) <= 1e-10f && canonicalNormal.Y < -1e-10f)
            || (Math.Abs(canonicalNormal.Z) <= 1e-10f && Math.Abs(canonicalNormal.Y) <= 1e-10f && canonicalNormal.X < 0);

        if (flip)
        {
            canonicalNormal = -canonicalNormal;
            canonicalDistance = -canonicalDistance;
        }

        const double precision = 10000.0;
        return new PlaneBucketKey(
            Nx: (int)Math.Round(canonicalNormal.X * precision),
            Ny: (int)Math.Round(canonicalNormal.Y * precision),
            Nz: (int)Math.Round(canonicalNormal.Z * precision),
            D: (int)Math.Round(canonicalDistance * precision));
    }

    private static PlaneBasis CreatePlaneBasis(Vector3 normal, double distance)
    {
        var up = SafeNormalize(normal, Vector3.UnitZ);
        var temp = Math.Abs(Vector3.Dot(up, Vector3.UnitY)) > 0.9f ? Vector3.UnitX : Vector3.UnitY;
        var axisX = SafeNormalize(Vector3.Cross(temp, up), Vector3.UnitX);
        var axisY = SafeNormalize(Vector3.Cross(up, axisX), Vector3.UnitY);
        var origin = up * (float)distance;
        return new PlaneBasis(origin, axisX, axisY);
    }

    private static List<Segment2> SplitSegmentsAtIntersections(IReadOnlyList<Segment2> segments)
    {
        var splitPoints = new List<List<Vector2>>(segments.Count);
        for (var i = 0; i < segments.Count; i++)
        {
            splitPoints.Add(new List<Vector2> { segments[i].A, segments[i].B });
        }

        for (var i = 0; i < segments.Count; i++)
        {
            for (var j = i + 1; j < segments.Count; j++)
            {
                if (!TryIntersectSegments2D(segments[i].A, segments[i].B, segments[j].A, segments[j].B, out var intersection))
                {
                    continue;
                }

                AddUniquePoint(splitPoints[i], intersection);
                AddUniquePoint(splitPoints[j], intersection);
            }
        }

        var output = new List<Segment2>();
        for (var i = 0; i < segments.Count; i++)
        {
            var points = splitPoints[i];
            if (points.Count < 2)
            {
                continue;
            }

            var segment = segments[i];
            var direction = segment.B - segment.A;
            points.Sort((left, right) => Vector2.Dot(left - segment.A, direction).CompareTo(Vector2.Dot(right - segment.A, direction)));

            for (var p = 1; p < points.Count; p++)
            {
                var a = points[p - 1];
                var b = points[p];
                if (Vector2.DistanceSquared(a, b) < 1e-12f)
                {
                    continue;
                }

                output.Add(new Segment2(a, b, segment.Source));
            }
        }

        return output;
    }

    private static bool TryIntersectSegments2D(Vector2 p1, Vector2 p2, Vector2 q1, Vector2 q2, out Vector2 intersection)
    {
        intersection = Vector2.Zero;
        var r = p2 - p1;
        var s = q2 - q1;
        var denom = Cross2D(r, s);

        if (Math.Abs(denom) < 1e-12f)
        {
            return false;
        }

        var qp = q1 - p1;
        var t = Cross2D(qp, s) / denom;
        var u = Cross2D(qp, r) / denom;

        if (t < -1e-6f || t > 1f + 1e-6f || u < -1e-6f || u > 1f + 1e-6f)
        {
            return false;
        }

        intersection = p1 + t * r;
        return true;
    }

    private static float Cross2D(Vector2 a, Vector2 b)
    {
        return a.X * b.Y - a.Y * b.X;
    }

    private static void AddUniquePoint(List<Vector2> points, Vector2 point)
    {
        for (var i = 0; i < points.Count; i++)
        {
            if (Vector2.DistanceSquared(points[i], point) < 1e-12f)
            {
                return;
            }
        }

        points.Add(point);
    }

    private static List<List<Vector2>> BuildClosedLoops(IReadOnlyList<Segment2> segments)
    {
        var pointMap = new Dictionary<string, int>(StringComparer.Ordinal);
        var points = new List<Vector2>();
        var edgeCounts = new Dictionary<(int A, int B), int>();

        int GetPointIndex(Vector2 point)
        {
            var key = CreatePoint2DKey(point);
            if (pointMap.TryGetValue(key, out var id))
            {
                return id;
            }

            id = points.Count;
            points.Add(point);
            pointMap[key] = id;
            return id;
        }

        for (var i = 0; i < segments.Count; i++)
        {
            var a = GetPointIndex(segments[i].A);
            var b = GetPointIndex(segments[i].B);
            if (a == b)
            {
                continue;
            }

            var edge = a < b ? (a, b) : (b, a);
            edgeCounts.TryGetValue(edge, out var count);
            edgeCounts[edge] = count + 1;
        }

        var adjacency = new Dictionary<int, List<int>>();
        foreach (var (edge, count) in edgeCounts)
        {
            if (count != 1)
            {
                continue;
            }

            if (!adjacency.TryGetValue(edge.A, out var listA))
            {
                listA = new List<int>();
                adjacency[edge.A] = listA;
            }

            if (!adjacency.TryGetValue(edge.B, out var listB))
            {
                listB = new List<int>();
                adjacency[edge.B] = listB;
            }

            listA.Add(edge.B);
            listB.Add(edge.A);
        }

        var unused = new HashSet<(int A, int B)>(edgeCounts.Where(kv => kv.Value == 1).Select(kv => kv.Key));
        var loops = new List<List<Vector2>>();

        while (unused.Count > 0)
        {
            var startEdge = unused.First();
            var start = startEdge.A;
            var next = startEdge.B;
            var loopIndices = new List<int> { start, next };
            unused.Remove(startEdge);

            var previous = start;
            var current = next;
            var guard = 0;
            while (guard++ < 10_000)
            {
                if (!adjacency.TryGetValue(current, out var neighbors) || neighbors.Count == 0)
                {
                    break;
                }

                var candidate = -1;
                for (var i = 0; i < neighbors.Count; i++)
                {
                    var neighbor = neighbors[i];
                    var edge = current < neighbor ? (current, neighbor) : (neighbor, current);
                    if (!unused.Contains(edge))
                    {
                        continue;
                    }

                    if (neighbor == previous && neighbors.Count > 1)
                    {
                        continue;
                    }

                    candidate = neighbor;
                    break;
                }

                if (candidate < 0)
                {
                    break;
                }

                var candidateEdge = current < candidate ? (current, candidate) : (candidate, current);
                unused.Remove(candidateEdge);
                previous = current;
                current = candidate;

                if (current == start)
                {
                    break;
                }

                loopIndices.Add(current);
            }

            if (loopIndices.Count >= 3 && current == start)
            {
                var loop = new List<Vector2>(loopIndices.Count);
                for (var i = 0; i < loopIndices.Count; i++)
                {
                    loop.Add(points[loopIndices[i]]);
                }

                loops.Add(loop);
            }
        }

        return loops;
    }

    private readonly record struct Tri2(int A, int B, int C);

    private static List<Tri2> TriangulatePolygon2D(IReadOnlyList<Vector2> polygon)
    {
        if (polygon.Count < 3)
        {
            return new List<Tri2>();
        }

        var indices = Enumerable.Range(0, polygon.Count).ToList();
        var triangles = new List<Tri2>();
        var ccw = SignedArea2D(polygon) >= 0;
        var guard = 0;
        var maxGuard = polygon.Count * polygon.Count;

        while (indices.Count > 3 && guard++ < maxGuard)
        {
            var earFound = false;
            for (var i = 0; i < indices.Count; i++)
            {
                var prev = indices[(i - 1 + indices.Count) % indices.Count];
                var curr = indices[i];
                var next = indices[(i + 1) % indices.Count];

                if (!IsConvex2D(polygon[prev], polygon[curr], polygon[next], ccw))
                {
                    continue;
                }

                var contains = false;
                for (var j = 0; j < indices.Count; j++)
                {
                    var p = indices[j];
                    if (p == prev || p == curr || p == next)
                    {
                        continue;
                    }

                    if (PointInTriangle2D(polygon[p], polygon[prev], polygon[curr], polygon[next]))
                    {
                        contains = true;
                        break;
                    }
                }

                if (contains)
                {
                    continue;
                }

                triangles.Add(new Tri2(prev, curr, next));
                indices.RemoveAt(i);
                earFound = true;
                break;
            }

            if (!earFound)
            {
                break;
            }
        }

        if (indices.Count == 3)
        {
            triangles.Add(new Tri2(indices[0], indices[1], indices[2]));
        }

        return triangles;
    }

    private static float SignedArea2D(IReadOnlyList<Vector2> polygon)
    {
        var area = 0f;
        for (var i = 0; i < polygon.Count; i++)
        {
            var j = (i + 1) % polygon.Count;
            area += polygon[i].X * polygon[j].Y - polygon[j].X * polygon[i].Y;
        }

        return area * 0.5f;
    }

    private static bool IsConvex2D(Vector2 prev, Vector2 curr, Vector2 next, bool ccw)
    {
        var cross = Cross2D(next - curr, prev - curr);
        return ccw ? cross > 1e-8f : cross < -1e-8f;
    }

    private static bool PointInTriangle2D(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
    {
        var d1 = Sign2D(p, a, b);
        var d2 = Sign2D(p, b, c);
        var d3 = Sign2D(p, c, a);
        var hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
        var hasPos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(hasNeg && hasPos);
    }

    private static float Sign2D(Vector2 p1, Vector2 p2, Vector2 p3)
    {
        return (p1.X - p3.X) * (p2.Y - p3.Y) - (p2.X - p3.X) * (p1.Y - p3.Y);
    }

    private static string CreatePoint2DKey(Vector2 point)
    {
        const double precision = 1_000_000d;
        var x = Math.Round(point.X * precision) / precision;
        var y = Math.Round(point.Y * precision) / precision;
        return string.Create(CultureInfo.InvariantCulture, $"{x:R},{y:R}");
    }

    private static string CreateUnorderedEdgeKey(Vector3 a, Vector3 b)
    {
        var ka = CreatePointKey(a);
        var kb = CreatePointKey(b);
        return string.CompareOrdinal(ka, kb) <= 0 ? $"{ka}|{kb}" : $"{kb}|{ka}";
    }

    private static InsideResult StableInsideCheck(Vector3 point, Vector3 normal, Geometry geometry, BoolBvh bvh, bool unionMode)
    {
        var directions = new[]
        {
            SafeNormalize(normal + new Vector3(0.02f, 0.01f, 0.04f), Vector3.UnitX),
            SafeNormalize(normal + new Vector3(0.20f, -0.10f, 0.40f), Vector3.UnitY),
            SafeNormalize(normal + new Vector3(-0.17f, 0.23f, -0.11f), Vector3.UnitZ)
        };

        Span<int> votes = stackalloc int[3];
        var boundary = new InsideResult(MeshLocation.Boundary, normal);

        foreach (var direction in directions)
        {
            var result = MeshClassifier.IsInsideMesh(point, normal, geometry, bvh, direction, unionMode);
            switch (result.Location)
            {
                case MeshLocation.Inside:
                    votes[0]++;
                    break;
                case MeshLocation.Outside:
                    votes[1]++;
                    break;
                case MeshLocation.Boundary:
                    votes[2]++;
                    if (result.Normal.LengthSquared() > 1e-10f)
                    {
                        boundary = result;
                    }

                    break;
            }
        }

        if (votes[0] > votes[1] && votes[0] >= votes[2])
        {
            return new InsideResult(MeshLocation.Inside, Vector3.Zero);
        }

        if (votes[1] > votes[0] && votes[1] >= votes[2])
        {
            return new InsideResult(MeshLocation.Outside, Vector3.Zero);
        }

        return boundary;
    }

    private static void AddUniqueFace(Geometry geometry, Vector3 a, Vector3 b, Vector3 c, int planeId, HashSet<string> seen)
    {
        var key = CreateUnorderedFaceKey(a, b, c);
        if (!seen.Add(key))
        {
            return;
        }

        geometry.AddFace(a, b, c, planeId);
    }

    private static string CreateUnorderedFaceKey(Vector3 a, Vector3 b, Vector3 c)
    {
        var keys = new[]
        {
            CreatePointKey(a),
            CreatePointKey(b),
            CreatePointKey(c)
        };

        Array.Sort(keys, StringComparer.Ordinal);
        return string.Concat(keys[0], "|", keys[1], "|", keys[2]);
    }

    private static string CreatePointKey(Vector3 point)
    {
        const double precision = 1_000_000d;
        var x = Math.Round(point.X * precision) / precision;
        var y = Math.Round(point.Y * precision) / precision;
        var z = Math.Round(point.Z * precision) / precision;
        return string.Create(
            CultureInfo.InvariantCulture,
            $"{x:R},{y:R},{z:R}");
    }

    private static Vector3 SafeNormalize(Vector3 v, Vector3 fallback)
    {
        if (v.LengthSquared() < 1e-12f)
        {
            return fallback;
        }

        return Vector3.Normalize(v);
    }
}
