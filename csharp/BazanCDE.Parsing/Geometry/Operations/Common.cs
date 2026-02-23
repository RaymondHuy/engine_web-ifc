using System.Numerics;

namespace BazanCDE.Parsing.Operations;

public static class Epsilons
{
    public static double ToleranceScalarEquality { get; set; } = 1.0E-04;
    public static double PlaneRefitIterations { get; set; } = 1;
    public static double BooleanUnionThreshold { get; set; } = 150;

    public const double EpsTinyCurve = 1.0E-09;
    public const double EpsNonZero = 1.0E-20;
    public const double EpsMiniscule = 1.0E-12;
    public const double EpsTiny = 1.0E-04;
    public const double EpsSmall = 1.0E-04;
    public const double EpsBig = 1.0E-04;
    public const double EpsBig2 = 1.0E-03;

    public const double ToleranceAddFace = 1.0E-10;
    public const double ToleranceVectorEquality = 1.0E-04;
    public const double ReconstructTolerance = 1.0E-01;
}

public sealed class Buffers
{
    public List<float> VertexData { get; } = new();
    public List<uint> IndexData { get; } = new();

    public void AddPoint(Vector3 point)
    {
        VertexData.Add(point.X);
        VertexData.Add(point.Y);
        VertexData.Add(point.Z);
    }

    public void AddTri(uint p1, uint p2, uint p3)
    {
        IndexData.Add(p1);
        IndexData.Add(p2);
        IndexData.Add(p3);
    }

    public void AddTri(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        AddPoint(p1);
        AddPoint(p2);
        AddPoint(p3);
        var id = (uint)(VertexData.Count / 3);
        AddTri(id - 3, id - 2, id - 1);
    }
}

public readonly record struct Face(int I0, int I1, int I2, int PlaneId);

public sealed class Plane
{
    public double Distance { get; set; }
    public Vector3 Normal { get; set; }
    public int Id { get; set; }

    public bool IsEqualTo(Vector3 normal, double distance)
    {
        return GeometryOps.Equals(Normal, normal, Epsilons.ToleranceVectorEquality)
            && Math.Abs(Distance - distance) <= Epsilons.ToleranceScalarEquality;
    }
}

public sealed class Aabb
{
    public uint Index { get; set; }
    public Vector3 Min { get; private set; } = new(float.MaxValue, float.MaxValue, float.MaxValue);
    public Vector3 Max { get; private set; } = new(float.MinValue, float.MinValue, float.MinValue);
    public Vector3 Center { get; private set; }

    public bool Intersects(Aabb other)
    {
        var eps = (float)Epsilons.EpsBig;
        return Max.X + eps >= other.Min.X && other.Max.X + eps >= Min.X
            && Max.Y + eps >= other.Min.Y && other.Max.Y + eps >= Min.Y
            && Max.Z + eps >= other.Min.Z && other.Max.Z + eps >= Min.Z;
    }

    public bool Contains(Vector3 point)
    {
        var eps = (float)Epsilons.EpsBig;
        return point.X + eps >= Min.X && point.X - eps <= Max.X
            && point.Y + eps >= Min.Y && point.Y - eps <= Max.Y
            && point.Z + eps >= Min.Z && point.Z - eps <= Max.Z;
    }

    public void Merge(Aabb other)
    {
        Min = Vector3.Min(Min, other.Min);
        Max = Vector3.Max(Max, other.Max);
        Center = (Min + Max) * 0.5f;
    }

    public void Merge(Vector3 point)
    {
        Min = Vector3.Min(Min, point);
        Max = Vector3.Max(Max, point);
        Center = (Min + Max) * 0.5f;
    }

    public bool IntersectRay(Vector3 origin, Vector3 direction)
    {
        var inv = new Vector3(
            direction.X == 0 ? float.PositiveInfinity : 1f / direction.X,
            direction.Y == 0 ? float.PositiveInfinity : 1f / direction.Y,
            direction.Z == 0 ? float.PositiveInfinity : 1f / direction.Z);

        var t1 = (Min.X - origin.X) * inv.X;
        var t2 = (Max.X - origin.X) * inv.X;
        var t3 = (Min.Y - origin.Y) * inv.Y;
        var t4 = (Max.Y - origin.Y) * inv.Y;
        var t5 = (Min.Z - origin.Z) * inv.Z;
        var t6 = (Max.Z - origin.Z) * inv.Z;

        var tMin = Math.Max(Math.Max(Math.Min(t1, t2), Math.Min(t3, t4)), Math.Min(t5, t6));
        var tMax = Math.Min(Math.Min(Math.Max(t1, t2), Math.Max(t3, t4)), Math.Max(t5, t6));

        if (tMax < -Epsilons.EpsBig)
        {
            return false;
        }

        return tMin <= tMax + Epsilons.EpsBig;
    }

    public void SetValues(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
    {
        Min = new Vector3((float)minX, (float)minY, (float)minZ);
        Max = new Vector3((float)maxX, (float)maxY, (float)maxZ);
        Center = (Min + Max) * 0.5f;
    }

    public Buffers GetBuffers()
    {
        var buffers = new Buffers();

        buffers.AddPoint(new Vector3(Max.X, Max.Y, Min.Z));
        buffers.AddPoint(new Vector3(Max.X, Min.Y, Min.Z));
        buffers.AddPoint(new Vector3(Max.X, Min.Y, Max.Z));
        buffers.AddPoint(new Vector3(Max.X, Max.Y, Max.Z));
        buffers.AddPoint(new Vector3(Min.X, Max.Y, Min.Z));
        buffers.AddPoint(new Vector3(Min.X, Min.Y, Min.Z));
        buffers.AddPoint(new Vector3(Min.X, Min.Y, Max.Z));
        buffers.AddPoint(new Vector3(Min.X, Max.Y, Max.Z));

        buffers.AddTri(0, 1, 3);
        buffers.AddTri(3, 1, 2);
        buffers.AddTri(5, 2, 1);
        buffers.AddTri(2, 5, 6);
        buffers.AddTri(7, 0, 4);
        buffers.AddTri(3, 0, 7);
        buffers.AddTri(7, 4, 5);
        buffers.AddTri(5, 6, 7);
        buffers.AddTri(6, 7, 3);
        buffers.AddTri(6, 2, 3);
        buffers.AddTri(5, 1, 4);
        buffers.AddTri(1, 0, 4);

        return buffers;
    }
}

public class Curve
{
    public List<Vector3> Points { get; } = new();

    public void Add(Vector3 point, bool removeCoincident = true)
    {
        if (Points.Count == 0 || !removeCoincident)
        {
            Points.Add(point);
            return;
        }

        if (!GeometryOps.Equals(Points[^1], point, Epsilons.EpsTinyCurve))
        {
            Points.Add(point);
        }
    }

    public void Add(Vector2 point)
    {
        Add(new Vector3(point, 0));
    }

    public void Invert()
    {
        Points.Reverse();
    }

    public bool IsCcw()
    {
        if (Points.Count < 3)
        {
            return false;
        }

        double sum = 0;
        var n = Points.Count;
        for (var i = 0; i < n; i++)
        {
            var p1 = Points[(i + n - 1) % n];
            var p2 = Points[i];
            sum += (p2.X - p1.X) * (p2.Y + p1.Y);
        }

        return sum < 0;
    }
}

public sealed class Geometry
{
    public const int VertexFormatSizeFloats = 6;

    public bool HasPlanes { get; private set; }
    public uint NumPoints => (uint)(VertexData.Count / VertexFormatSizeFloats);
    public uint NumFaces => (uint)(IndexData.Count / 3);

    public List<float> FVertexData { get; } = new();
    public List<double> VertexData { get; } = new();
    public List<uint> IndexData { get; } = new();
    public List<int> PlaneData { get; } = new();
    public List<Plane> Planes { get; } = new();

    public Aabb GetAabb()
    {
        var aabb = new Aabb();
        for (var i = 0u; i < NumPoints; i++)
        {
            aabb.Merge(GetPoint(i));
        }

        return aabb;
    }

    public Aabb GetFaceBox(uint index)
    {
        var face = GetFace(index);
        var box = new Aabb { Index = index };
        box.Merge(GetPoint((uint)face.I0));
        box.Merge(GetPoint((uint)face.I1));
        box.Merge(GetPoint((uint)face.I2));
        return box;
    }

    public Vector3 GetPoint(uint index)
    {
        var o = (int)index * VertexFormatSizeFloats;
        return new Vector3((float)VertexData[o], (float)VertexData[o + 1], (float)VertexData[o + 2]);
    }

    public void SetPoint(double x, double y, double z, uint index)
    {
        var o = (int)index * VertexFormatSizeFloats;
        VertexData[o] = x;
        VertexData[o + 1] = y;
        VertexData[o + 2] = z;
    }

    public Face GetFace(uint index)
    {
        var o = (int)index * 3;
        var plane = index < PlaneData.Count ? PlaneData[(int)index] : -1;
        return new Face((int)IndexData[o], (int)IndexData[o + 1], (int)IndexData[o + 2], plane);
    }

    public void AddPoint(Vector3 point, Vector3 normal)
    {
        VertexData.Add(point.X);
        VertexData.Add(point.Y);
        VertexData.Add(point.Z);
        VertexData.Add(normal.X);
        VertexData.Add(normal.Y);
        VertexData.Add(normal.Z);
    }

    public void AddFace(Vector3 a, Vector3 b, Vector3 c, int planeId = -1)
    {
        if (!GeometryOps.ComputeSafeNormal(a, b, c, out var normal, Epsilons.ToleranceAddFace))
        {
            return;
        }

        AddPoint(a, normal);
        AddPoint(b, normal);
        AddPoint(c, normal);

        var n = NumPoints;
        AddFace(n - 3, n - 2, n - 1, planeId);
    }

    public void AddFace(uint a, uint b, uint c, int planeId = -1)
    {
        IndexData.Add(a);
        IndexData.Add(b);
        IndexData.Add(c);
        PlaneData.Add(planeId);
    }

    public int AddPlane(Vector3 normal, double distance)
    {
        for (var i = 0; i < Planes.Count; i++)
        {
            if (Planes[i].IsEqualTo(normal, distance))
            {
                return Planes[i].Id;
            }
        }

        var p = new Plane
        {
            Id = Planes.Count,
            Normal = Vector3.Normalize(normal),
            Distance = distance
        };

        Planes.Add(p);
        return p.Id;
    }

    public void BuildPlanes()
    {
        if (HasPlanes || NumFaces == 0)
        {
            return;
        }

        Planes.Clear();
        PlaneData.Clear();

        var centroid = Vector3.Zero;
        for (var i = 0u; i < NumFaces; i++)
        {
            var f = GetFace(i);
            centroid += (GetPoint((uint)f.I0) + GetPoint((uint)f.I1) + GetPoint((uint)f.I2)) / 3f;
        }

        centroid /= NumFaces;

        for (var i = 0u; i < NumFaces; i++)
        {
            var f = GetFace(i);
            var a = GetPoint((uint)f.I0);
            var b = GetPoint((uint)f.I1);
            var c = GetPoint((uint)f.I2);

            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                PlaneData.Add(-1);
                continue;
            }

            var da = Vector3.Dot(n, a - centroid);
            var db = Vector3.Dot(n, b - centroid);
            var dc = Vector3.Dot(n, c - centroid);
            var id = AddPlane(n, (da + db + dc) / 3.0);
            PlaneData.Add(id);
        }

        HasPlanes = true;
    }

    public void AddGeometry(Geometry geometry)
    {
        for (var i = 0u; i < geometry.NumFaces; i++)
        {
            var f = geometry.GetFace(i);
            AddFace(
                geometry.GetPoint((uint)f.I0),
                geometry.GetPoint((uint)f.I1),
                geometry.GetPoint((uint)f.I2));
        }
    }

    public bool IsEmpty()
    {
        return VertexData.Count == 0 || IndexData.Count == 0;
    }

    public void GetCenterExtents(out Vector3 center, out Vector3 extents)
    {
        if (NumPoints == 0)
        {
            center = Vector3.Zero;
            extents = Vector3.Zero;
            return;
        }

        var min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        var max = new Vector3(float.MinValue, float.MinValue, float.MinValue);

        for (var i = 0u; i < NumPoints; i++)
        {
            var pt = GetPoint(i);
            min = Vector3.Min(min, pt);
            max = Vector3.Max(max, pt);
        }

        extents = max - min;
        center = min + extents * 0.5f;
    }

    public Geometry Normalize(Vector3 center, Vector3 extents)
    {
        var normalized = new Geometry();
        var scale = Math.Max(extents.X, Math.Max(extents.Y, extents.Z)) / 10.0f;
        if (Math.Abs(scale) < Epsilons.EpsNonZero)
        {
            scale = 1.0f;
        }

        for (var i = 0u; i < NumFaces; i++)
        {
            var face = GetFace(i);
            var a = (GetPoint((uint)face.I0) - center) / scale;
            var b = (GetPoint((uint)face.I1) - center) / scale;
            var c = (GetPoint((uint)face.I2) - center) / scale;
            normalized.AddFace(a, b, c, face.PlaneId);
        }

        return normalized;
    }

    public Geometry Denormalize(Vector3 center, Vector3 extents)
    {
        var denormalized = new Geometry();
        var scale = Math.Max(extents.X, Math.Max(extents.Y, extents.Z)) / 10.0f;
        if (Math.Abs(scale) < Epsilons.EpsNonZero)
        {
            scale = 1.0f;
        }

        for (var i = 0u; i < NumFaces; i++)
        {
            var face = GetFace(i);
            var a = GetPoint((uint)face.I0) * scale + center;
            var b = GetPoint((uint)face.I1) * scale + center;
            var c = GetPoint((uint)face.I2) * scale + center;
            denormalized.AddFace(a, b, c, face.PlaneId);
        }

        return denormalized;
    }

    public double Volume(Matrix4x4? transform = null)
    {
        var t = transform ?? Matrix4x4.Identity;
        double total = 0;

        for (var i = 0u; i < NumFaces; i++)
        {
            var face = GetFace(i);
            var a = Vector3.Transform(GetPoint((uint)face.I0), t);
            var b = Vector3.Transform(GetPoint((uint)face.I1), t);
            var c = Vector3.Transform(GetPoint((uint)face.I2), t);

            if (!GeometryOps.ComputeSafeNormal(a, b, c, out var n, Epsilons.EpsSmall))
            {
                continue;
            }

            var area = GeometryOps.AreaOfTriangle(a, b, c);
            var height = Vector3.Dot(n, a);
            total += area * height / 3.0;
        }

        return total;
    }
}
