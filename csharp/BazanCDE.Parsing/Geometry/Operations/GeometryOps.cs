using System.Numerics;

namespace BazanCDE.Parsing.Operations;

public static class GeometryOps
{
    public const double ConstPi = Math.PI;

    public static bool Equals(Vector3 a, Vector3 b, double eps = 0)
    {
        return Math.Abs(a.X - b.X) <= eps
            && Math.Abs(a.Y - b.Y) <= eps
            && Math.Abs(a.Z - b.Z) <= eps;
    }

    public static bool Equals(double a, double b, double eps = 0)
    {
        return Math.Abs(a - b) <= eps;
    }

    public static double AreaOfTriangle(Vector3 a, Vector3 b, Vector3 c)
    {
        var ab = b - a;
        var ac = c - a;
        var n = Vector3.Cross(ab, ac);
        return n.Length() * 0.5;
    }

    public static bool ComputeSafeNormal(Vector3 v1, Vector3 v2, Vector3 v3, out Vector3 normal, double eps = 0)
    {
        var n = Vector3.Cross(v2 - v1, v3 - v1);
        var len = n.Length();
        if (len <= eps)
        {
            normal = Vector3.Zero;
            return false;
        }

        normal = n / len;
        return true;
    }

    public static double VectorToAngle(double x, double y)
    {
        var dd = Math.Sqrt(x * x + y * y);
        if (Math.Abs(dd) < Epsilons.EpsMiniscule)
        {
            return 0;
        }

        var xx = x / dd;
        var yy = y / dd;

        var angle = Math.Acos(Math.Clamp(xx, -1, 1));
        var cosv = Math.Cos(angle);
        var sinv = Math.Sin(angle);
        if (Math.Abs(xx - cosv) > 1e-5 || Math.Abs(yy - sinv) > 1e-5)
        {
            angle = Math.Asin(Math.Clamp(yy, -1, 1));
            sinv = Math.Sin(angle);
            cosv = Math.Cos(angle);
            if (Math.Abs(xx - cosv) > 1e-5 || Math.Abs(yy - sinv) > 1e-5)
            {
                angle = angle + (ConstPi - angle) * 2;
            }
        }

        return angle / (2 * ConstPi) * 360;
    }

    public static Curve GetEllipseCurve(
        double radiusX,
        double radiusY,
        int segments,
        double[]? placement = null,
        double startRad = 0,
        double endRad = ConstPi * 2,
        bool swap = true,
        bool normalToCenterEnding = false)
    {
        _ = normalToCenterEnding;

        segments = Math.Max(3, segments);
        var span = endRad - startRad;

        var mat = placement is { Length: 9 }
            ? placement
            : new[]
            {
                1d, 0d, 0d,
                0d, 1d, 0d,
                0d, 0d, 1d
            };

        var curve = new Curve();
        for (var i = 0; i <= segments; i++)
        {
            var t = startRad + span * i / segments;
            var x = radiusX * (swap ? Math.Sin(t) : Math.Cos(t));
            var y = radiusY * (swap ? Math.Cos(t) : Math.Sin(t));

            var p = Transform3x3(new Vector3((float)x, (float)y, 0), mat);
            curve.Add(p);
        }

        if (curve.Points.Count > 1 && Equals(curve.Points[0], curve.Points[^1], Epsilons.EpsTinyCurve))
        {
            curve.Points.RemoveAt(curve.Points.Count - 1);
        }

        return curve;
    }

    public static List<Vector2> SolveParabola(
        int segments,
        Vector2 startPoint,
        double horizontalLength,
        double startHeight,
        double startGradient,
        double endGradient)
    {
        segments = Math.Max(2, segments);
        var result = new List<Vector2>(segments + 1);

        var h = Math.Abs(horizontalLength) < Epsilons.EpsNonZero ? 1.0 : horizontalLength;
        var a = (endGradient - startGradient) / (2.0 * h);
        var b = startGradient;
        var c = startHeight;

        for (var i = 0; i <= segments; i++)
        {
            var x = horizontalLength * i / segments;
            var y = a * x * x + b * x + c;
            result.Add(new Vector2((float)(startPoint.X + x), (float)(startPoint.Y + y)));
        }

        return result;
    }

    public static List<Vector2> SolveClothoid(
        int segments,
        Vector2 startPoint,
        double startDirection,
        double startRadius,
        double endRadius,
        double segmentLength)
    {
        segments = Math.Max(4, segments);
        var points = new List<Vector2>(segments + 1) { startPoint };

        var length = Math.Max(segmentLength, Epsilons.EpsNonZero);
        var ds = length / segments;

        var k0 = Math.Abs(startRadius) < Epsilons.EpsNonZero ? 0 : 1.0 / startRadius;
        var k1 = Math.Abs(endRadius) < Epsilons.EpsNonZero ? 0 : 1.0 / endRadius;

        var theta = startDirection;
        var p = startPoint;

        for (var i = 1; i <= segments; i++)
        {
            var s = ds * (i - 0.5);
            var k = k0 + (k1 - k0) * (s / length);
            theta += k * ds;

            p = new Vector2(
                p.X + (float)(Math.Cos(theta) * ds),
                p.Y + (float)(Math.Sin(theta) * ds));
            points.Add(p);
        }

        return points;
    }

    public static List<Vector3> Convert2DAlignmentsTo3D(IReadOnlyList<Vector3> horizontal, IReadOnlyList<Vector3> vertical)
    {
        if (horizontal.Count == 0)
        {
            return new List<Vector3>();
        }

        if (vertical.Count == 0)
        {
            return horizontal.ToList();
        }

        var result = new List<Vector3>(horizontal.Count);

        for (var i = 0; i < horizontal.Count; i++)
        {
            var h = horizontal[i];
            var elevation = SampleVerticalElevation((float)h.X, vertical);
            result.Add(new Vector3(h.X, elevation, h.Y));
        }

        return result;
    }

    public static Geometry Revolution(Matrix4x4 transform, double startDegrees, double endDegrees, IReadOnlyList<Vector3> profile, double numRots)
    {
        var geometry = new Geometry();
        if (profile.Count < 2)
        {
            return geometry;
        }

        var steps = Math.Max(2, (int)Math.Round(numRots));

        var origin = new Vector3(transform.M41, transform.M42, transform.M43);
        var vecX = SafeNormalize(new Vector3(transform.M11, transform.M12, transform.M13), Vector3.UnitX);
        var vecY = SafeNormalize(new Vector3(transform.M21, transform.M22, transform.M23), Vector3.UnitY);
        var vecZ = SafeNormalize(new Vector3(transform.M31, transform.M32, transform.M33), Vector3.UnitZ);

        var startRad = DegreesToRadians(startDegrees);
        var endRad = DegreesToRadians(endDegrees);
        var stepRad = (endRad - startRad) / (steps - 1);

        var grid = new List<List<Vector3>>(steps);
        for (var i = 0; i < steps; i++)
        {
            grid.Add(new List<Vector3>(profile.Count));
        }

        foreach (var p in profile)
        {
            var d = p - origin;
            var dx = Vector3.Dot(vecX, d);
            var dy = Vector3.Dot(vecY, d);
            var dz = Vector3.Dot(vecZ, d);
            var radius = Math.Sqrt(dx * dx + dy * dy);

            for (var r = 0; r < steps; r++)
            {
                var angle = startRad + stepRad * r;
                var px = (float)(Math.Sin(angle) * radius);
                var py = (float)(Math.Cos(angle) * radius);
                var world = origin + vecX * px + vecY * py + vecZ * dz;
                grid[r].Add(world);
            }
        }

        StitchGrid(geometry, grid, closeU: false, closeV: false);
        return geometry;
    }

    public static Geometry RevolveCylinder(Matrix4x4 transform, double startDegrees, double endDegrees, double minZ, double maxZ, int numRots, double radius)
    {
        var profile = new List<Vector3>
        {
            new(0, 0, (float)minZ),
            new(0, 0, (float)maxZ)
        };

        var geom = Revolution(transform, startDegrees, endDegrees, profile.Select(p => p + new Vector3((float)radius, 0, 0)).ToList(), numRots);
        return geom;
    }

    public static Geometry Extrude(IReadOnlyList<Vector3> profile, Vector3 dir, double len, bool cap = true)
    {
        var geometry = new Geometry();

        var ring = RemoveDuplicateClosure(profile);
        if (ring.Count < 3)
        {
            return geometry;
        }

        var extrude = SafeNormalize(dir, Vector3.UnitZ) * (float)len;
        var top = ring.Select(p => p + extrude).ToList();

        for (var i = 0; i < ring.Count; i++)
        {
            var j = (i + 1) % ring.Count;
            geometry.AddFace(ring[i], ring[j], top[j]);
            geometry.AddFace(ring[i], top[j], top[i]);
        }

        if (cap)
        {
            var tris = TriangulatePolygon(ring);
            foreach (var tri in tris)
            {
                geometry.AddFace(ring[tri.A], ring[tri.C], ring[tri.B]);
                geometry.AddFace(top[tri.A], top[tri.B], top[tri.C]);
            }
        }

        return geometry;
    }

    public static Geometry Extrude(IReadOnlyList<IReadOnlyList<Vector3>> profiles, Vector3 dir, double len, Vector3 cuttingPlaneNormal, Vector3 cuttingPlanePos)
    {
        _ = cuttingPlaneNormal;
        _ = cuttingPlanePos;

        var result = new Geometry();
        if (profiles.Count == 0)
        {
            return result;
        }

        // First profile is treated as outer boundary, others as holes with inverted side winding.
        var outer = Extrude(profiles[0], dir, len, cap: true);
        result.AddGeometry(outer);

        for (var i = 1; i < profiles.Count; i++)
        {
            var holeRing = RemoveDuplicateClosure(profiles[i]);
            if (holeRing.Count < 3)
            {
                continue;
            }

            var extrude = SafeNormalize(dir, Vector3.UnitZ) * (float)len;
            var top = holeRing.Select(p => p + extrude).ToList();

            for (var p = 0; p < holeRing.Count; p++)
            {
                var n = (p + 1) % holeRing.Count;
                result.AddFace(holeRing[p], top[n], holeRing[n]);
                result.AddFace(holeRing[p], top[p], top[n]);
            }
        }

        return result;
    }

    public static Geometry SweepFunction(
        double scaling,
        bool closed,
        IReadOnlyList<Vector3> profilePoints,
        IReadOnlyList<Vector3> directrix,
        Vector3 initialDirectrixNormal,
        bool rotate90,
        bool optimize)
    {
        _ = optimize;

        var geometry = new Geometry();
        var ring = RemoveDuplicateClosure(profilePoints);
        if (ring.Count < 3 || directrix.Count < 2)
        {
            return geometry;
        }

        var sections = BuildSweepSections((float)scaling, closed, ring, directrix, initialDirectrixNormal, rotate90);
        StitchGrid(geometry, sections, closeU: closed, closeV: true);
        return geometry;
    }

    public static Geometry SweepCircular(
        double scaling,
        bool closed,
        IReadOnlyList<Vector3> profilePoints,
        double radius,
        IReadOnlyList<Vector3> directrix,
        Vector3 initialDirectrixNormal,
        bool rotate90)
    {
        var ring = RemoveDuplicateClosure(profilePoints);
        if (ring.Count < 3)
        {
            ring = GetEllipseCurve(radius, radius, 16).Points;
        }

        var normalized = new List<Vector3>(ring.Count);
        foreach (var p in ring)
        {
            normalized.Add(new Vector3(p.X * (float)radius, p.Y * (float)radius, p.Z));
        }

        return SweepFunction(scaling, closed, normalized, directrix, initialDirectrixNormal, rotate90, optimize: false);
    }

    public static Geometry Union(Geometry first, Geometry second)
    {
        first.BuildPlanes();
        second.BuildPlanes();
        return BooleanCsg.Union(first, second);
    }

    public static Geometry Subtract(Geometry first, Geometry second)
    {
        first.BuildPlanes();
        second.BuildPlanes();
        return BooleanCsg.Subtract(first, second);
    }

    public static Geometry Intersect(Geometry first, Geometry second)
    {
        first.BuildPlanes();
        second.BuildPlanes();
        return BooleanCsg.Intersect(first, second);
    }

    public static Geometry BoolProcess(Geometry firstOperator, List<Geometry> secondGeometries, string op)
    {
        var current = firstOperator;
        var normalizedOp = NormalizeBooleanOp(op);

        foreach (var second in secondGeometries)
        {
            if (second.NumFaces == 0)
            {
                continue;
            }

            if (current.NumFaces == 0 && normalizedOp != "UNION")
            {
                break;
            }

            current = normalizedOp switch
            {
                "UNION" => Union(current, second),
                "INTERSECTION" => Intersect(current, second),
                _ => Subtract(current, second)
            };
        }

        var final = new Geometry();
        final.AddGeometry(current);
        return final;
    }

    private static string NormalizeBooleanOp(string op)
    {
        var normalized = op.Trim();
        if (normalized.StartsWith(".", StringComparison.Ordinal) && normalized.EndsWith(".", StringComparison.Ordinal))
        {
            normalized = normalized.Trim('.');
        }

        return normalized.ToUpperInvariant();
    }

    public static Curve GetIShapedCurve(double width, double depth, double webThickness, double flangeThickness, bool hasFillet, double filletRadius, Matrix4x4 placement)
    {
        _ = hasFillet;
        _ = filletRadius;
        var hw = (float)(width / 2.0);
        var hd = (float)(depth / 2.0);
        var tw = (float)(webThickness / 2.0);
        var tf = (float)flangeThickness;

        var points = new List<Vector3>
        {
            new(-hw, -hd, 0),
            new(hw, -hd, 0),
            new(hw, -hd + tf, 0),
            new(tw, -hd + tf, 0),
            new(tw, hd - tf, 0),
            new(hw, hd - tf, 0),
            new(hw, hd, 0),
            new(-hw, hd, 0),
            new(-hw, hd - tf, 0),
            new(-tw, hd - tf, 0),
            new(-tw, -hd + tf, 0),
            new(-hw, -hd + tf, 0)
        };

        return TransformCurve(points, placement);
    }

    public static Curve GetCShapedCurve(double width, double depth, double webThickness, double flangeThickness, bool hasFillet, double filletRadius, Matrix4x4 placement)
    {
        _ = hasFillet;
        _ = filletRadius;
        var hw = (float)(width / 2.0);
        var hd = (float)(depth / 2.0);
        var tw = (float)webThickness;
        var tf = (float)flangeThickness;

        var points = new List<Vector3>
        {
            new(-hw, -hd, 0),
            new(hw, -hd, 0),
            new(hw, -hd + tf, 0),
            new(-hw + tw, -hd + tf, 0),
            new(-hw + tw, hd - tf, 0),
            new(hw, hd - tf, 0),
            new(hw, hd, 0),
            new(-hw, hd, 0)
        };

        return TransformCurve(points, placement);
    }

    public static Curve GetZShapedCurve(double width, double depth, double webThickness, double flangeThickness, bool hasFillet, double filletRadius, Matrix4x4 placement)
    {
        _ = hasFillet;
        _ = filletRadius;
        var hw = (float)(width / 2.0);
        var hd = (float)(depth / 2.0);
        var tw = (float)webThickness;
        var tf = (float)flangeThickness;

        var points = new List<Vector3>
        {
            new(-hw, -hd, 0),
            new(hw, -hd, 0),
            new(hw, -hd + tf, 0),
            new(tw, -hd + tf, 0),
            new(-hw, hd - tf, 0),
            new(-hw, hd, 0),
            new(hw, hd, 0),
            new(hw, hd - tf, 0),
            new(-tw, hd - tf, 0),
            new(hw, -hd + tf, 0)
        };

        return TransformCurve(points, placement);
    }

    public static Curve GetTShapedCurve(double width, double depth, double webThickness, bool hasFillet, double filletRadius, double radius, double slope, Matrix4x4 placement)
    {
        _ = hasFillet;
        _ = filletRadius;
        _ = radius;
        _ = slope;

        var hw = (float)(width / 2.0);
        var hd = (float)(depth / 2.0);
        var tw = (float)(webThickness / 2.0);

        var points = new List<Vector3>
        {
            new(-hw, hd, 0),
            new(hw, hd, 0),
            new(hw, hd - (float)(depth * 0.25), 0),
            new(tw, hd - (float)(depth * 0.25), 0),
            new(tw, -hd, 0),
            new(-tw, -hd, 0),
            new(-tw, hd - (float)(depth * 0.25), 0),
            new(-hw, hd - (float)(depth * 0.25), 0)
        };

        return TransformCurve(points, placement);
    }

    public static Curve GetLShapedCurve(double width, double depth, double thickness, bool hasFillet, double filletRadius, double radius, double slope, int numSegments, Matrix4x4 placement)
    {
        _ = hasFillet;
        _ = filletRadius;
        _ = radius;
        _ = slope;
        _ = numSegments;

        var w = (float)width;
        var d = (float)depth;
        var t = (float)thickness;

        var points = new List<Vector3>
        {
            new(0, 0, 0),
            new(w, 0, 0),
            new(w, t, 0),
            new(t, t, 0),
            new(t, d, 0),
            new(0, d, 0)
        };

        return TransformCurve(points, placement);
    }

    public static Curve GetUShapedCurve(double width, double depth, double webThickness, double flangeThickness, double filletRadius, double radius, double slope, Matrix4x4 placement)
    {
        _ = filletRadius;
        _ = radius;
        _ = slope;

        var hw = (float)(width / 2.0);
        var hd = (float)(depth / 2.0);
        var tw = (float)webThickness;
        var tf = (float)flangeThickness;

        var points = new List<Vector3>
        {
            new(-hw, -hd, 0),
            new(hw, -hd, 0),
            new(hw, hd, 0),
            new(hw - tf, hd, 0),
            new(hw - tf, -hd + tw, 0),
            new(-hw + tf, -hd + tw, 0),
            new(-hw + tf, hd, 0),
            new(-hw, hd, 0)
        };

        return TransformCurve(points, placement);
    }

    public static bool MatrixFlipsTriangles(Matrix4x4 matrix)
    {
        return matrix.GetDeterminant() < 0;
    }

    private static Curve TransformCurve(IReadOnlyList<Vector3> points, Matrix4x4 placement)
    {
        var curve = new Curve();
        for (var i = 0; i < points.Count; i++)
        {
            curve.Add(Vector3.Transform(points[i], placement));
        }

        return curve;
    }

    private static Vector3 Transform3x3(Vector3 p, IReadOnlyList<double> m)
    {
        // Simple row-major 3x3 transform with implicit z.
        return new Vector3(
            (float)(m[0] * p.X + m[1] * p.Y + m[2]),
            (float)(m[3] * p.X + m[4] * p.Y + m[5]),
            (float)(m[6] * p.X + m[7] * p.Y + m[8]));
    }

    private static float SampleVerticalElevation(float x, IReadOnlyList<Vector3> vertical)
    {
        if (vertical.Count == 1)
        {
            return vertical[0].Y;
        }

        var prev = vertical[0];
        for (var i = 1; i < vertical.Count; i++)
        {
            var next = vertical[i];
            if ((x >= prev.X && x <= next.X) || (x <= prev.X && x >= next.X))
            {
                var span = next.X - prev.X;
                if (Math.Abs(span) < Epsilons.EpsNonZero)
                {
                    return prev.Y;
                }

                var t = (x - prev.X) / span;
                return prev.Y + (next.Y - prev.Y) * t;
            }

            prev = next;
        }

        return vertical[^1].Y;
    }

    private static List<List<Vector3>> BuildSweepSections(
        float scaling,
        bool closed,
        IReadOnlyList<Vector3> profile,
        IReadOnlyList<Vector3> directrix,
        Vector3 initialNormal,
        bool rotate90)
    {
        var sections = new List<List<Vector3>>(directrix.Count);

        var normal = SafeNormalize(initialNormal, Vector3.UnitY);
        var binormal = Vector3.UnitX;

        for (var i = 0; i < directrix.Count; i++)
        {
            var prev = directrix[Math.Max(0, i - 1)];
            var next = directrix[Math.Min(directrix.Count - 1, i + 1)];
            var tangent = SafeNormalize(next - prev, Vector3.UnitZ);

            binormal = Vector3.Cross(normal, tangent);
            if (binormal.LengthSquared() < 1e-10f)
            {
                binormal = Vector3.Cross(Math.Abs(tangent.X) > 0.9f ? Vector3.UnitY : Vector3.UnitX, tangent);
            }

            binormal = SafeNormalize(binormal, Vector3.UnitX);
            normal = SafeNormalize(Vector3.Cross(tangent, binormal), Vector3.UnitY);

            var xAxis = rotate90 ? normal : binormal;
            var yAxis = rotate90 ? binormal : normal;

            var section = new List<Vector3>(profile.Count);
            for (var p = 0; p < profile.Count; p++)
            {
                var lp = profile[p] * scaling;
                var world = directrix[i] + xAxis * lp.X + yAxis * lp.Y + tangent * lp.Z;
                section.Add(world);
            }

            sections.Add(section);
        }

        return sections;
    }

    private static void StitchGrid(Geometry geometry, IReadOnlyList<List<Vector3>> grid, bool closeU, bool closeV)
    {
        if (grid.Count < 2)
        {
            return;
        }

        var sectionCount = grid.Count;
        var pointCount = grid[0].Count;
        if (pointCount < 2)
        {
            return;
        }

        var maxU = closeU ? sectionCount : sectionCount - 1;
        var maxV = closeV ? pointCount : pointCount - 1;

        for (var u = 0; u < maxU; u++)
        {
            var nu = (u + 1) % sectionCount;
            if (!closeU && u + 1 >= sectionCount)
            {
                break;
            }

            for (var v = 0; v < maxV; v++)
            {
                var nv = (v + 1) % pointCount;
                if (!closeV && v + 1 >= pointCount)
                {
                    break;
                }

                var a = grid[u][v];
                var b = grid[u][nv];
                var c = grid[nu][nv];
                var d = grid[nu][v];

                geometry.AddFace(a, b, c);
                geometry.AddFace(a, c, d);
            }
        }
    }

    public readonly record struct Int3(int A, int B, int C);

    private static List<Int3> TriangulatePolygon(IReadOnlyList<Vector3> polygon)
    {
        if (polygon.Count < 3)
        {
            return new List<Int3>();
        }

        if (!ComputeSafeNormal(polygon[0], polygon[1], polygon[2], out var n, 1e-12))
        {
            n = Vector3.UnitZ;
        }

        var projected = ProjectTo2D(polygon, n);
        var order = Enumerable.Range(0, polygon.Count).ToList();
        var triangles = new List<Int3>();
        var ccw = SignedArea(projected) >= 0;
        var guard = 0;
        var maxGuard = polygon.Count * polygon.Count;

        while (order.Count > 3 && guard < maxGuard)
        {
            guard++;
            var found = false;

            for (var i = 0; i < order.Count; i++)
            {
                var prev = order[(i - 1 + order.Count) % order.Count];
                var curr = order[i];
                var next = order[(i + 1) % order.Count];

                if (!IsConvex(projected[prev], projected[curr], projected[next], ccw))
                {
                    continue;
                }

                var contains = false;
                for (var j = 0; j < order.Count; j++)
                {
                    var candidate = order[j];
                    if (candidate == prev || candidate == curr || candidate == next)
                    {
                        continue;
                    }

                    if (PointInTriangle(projected[candidate], projected[prev], projected[curr], projected[next]))
                    {
                        contains = true;
                        break;
                    }
                }

                if (contains)
                {
                    continue;
                }

                triangles.Add(new Int3(prev, curr, next));
                order.RemoveAt(i);
                found = true;
                break;
            }

            if (!found)
            {
                break;
            }
        }

        if (order.Count == 3)
        {
            triangles.Add(new Int3(order[0], order[1], order[2]));
        }

        if (!ccw)
        {
            for (var i = 0; i < triangles.Count; i++)
            {
                var t = triangles[i];
                triangles[i] = new Int3(t.A, t.C, t.B);
            }
        }

        return triangles;
    }

    private static List<Vector2> ProjectTo2D(IReadOnlyList<Vector3> polygon, Vector3 normal)
    {
        var ax = Math.Abs(normal.X);
        var ay = Math.Abs(normal.Y);
        var az = Math.Abs(normal.Z);

        var projected = new List<Vector2>(polygon.Count);
        for (var i = 0; i < polygon.Count; i++)
        {
            var p = polygon[i];
            if (az >= ax && az >= ay)
            {
                projected.Add(new Vector2(p.X, p.Y));
            }
            else if (ax >= ay)
            {
                projected.Add(new Vector2(p.Y, p.Z));
            }
            else
            {
                projected.Add(new Vector2(p.X, p.Z));
            }
        }

        return projected;
    }

    private static double SignedArea(IReadOnlyList<Vector2> polygon)
    {
        double area = 0;
        for (var i = 0; i < polygon.Count; i++)
        {
            var j = (i + 1) % polygon.Count;
            area += polygon[i].X * polygon[j].Y - polygon[j].X * polygon[i].Y;
        }

        return area * 0.5;
    }

    private static bool IsConvex(Vector2 prev, Vector2 curr, Vector2 next, bool ccw)
    {
        var cross = Cross(next - curr, prev - curr);
        return ccw ? cross > 1e-8 : cross < -1e-8;
    }

    private static bool PointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
    {
        var ab = Sign(p, a, b);
        var bc = Sign(p, b, c);
        var ca = Sign(p, c, a);

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

    private static List<Vector3> RemoveDuplicateClosure(IReadOnlyList<Vector3> points)
    {
        var result = points.ToList();
        if (result.Count > 1 && Equals(result[0], result[^1], Epsilons.EpsTinyCurve))
        {
            result.RemoveAt(result.Count - 1);
        }

        return result;
    }

    private static Vector3 SafeNormalize(Vector3 vector, Vector3 fallback)
    {
        var len = vector.LengthSquared();
        return len < 1e-12f ? fallback : Vector3.Normalize(vector);
    }

    private static double DegreesToRadians(double degrees)
    {
        return degrees / 180.0 * ConstPi;
    }
}
