using System;
using System.Collections.Generic;
using System.Numerics;

namespace BazanCDE.Parsing.Operations.BooleanUtils
{
    public readonly record struct SimplePlane(Vec3 Normal, double Distance, uint Id = 0);
    public readonly record struct Face(uint I0, uint I1, uint I2, uint PId);
    public class Geometry
    {
        public const int VERTEX_FORMAT_SIZE_FLOATS = 6;

        public List<float> FVertexData { get; } = new();
        public List<double> VertexData { get; private set; } = new();
        public List<uint> IndexData { get; private set; } = new();
        public List<uint> PlaneData { get; } = new();
        public List<SimplePlane> Planes { get; } = new();

        public bool HasPlanes { get; set; } = false;
        public uint NumPoints { get; private set; } = 0;
        public uint NumFaces { get; private set; } = 0;
        public uint Data { get; set; } = 0;

        public void BuildFromVectors(List<double> d, List<uint> i)
        {
            VertexData = new List<double>(d);
            IndexData = new List<uint>(i);

            // Kept equivalent to C++ implementation.
            NumPoints = (uint)IndexData.Count;
            NumFaces = (uint)(IndexData.Count / 3);
        }

        public void AddPoint(Vec4 pt, Vec3 n)
        {
            AddPoint(pt.XYZ, n);
        }

        public AABB GetAABB()
        {
            var aabb = new AABB();

            for (var i = 0u; i < NumPoints; i++)
            {
                aabb.Min = Vec3.Min(aabb.Min, GetPoint(i));
                aabb.Max = Vec3.Max(aabb.Max, GetPoint(i));
            }

            return aabb;
        }

        public void AddPoint(Vec3 pt, Vec3 n)
        {
            VertexData.Add(pt.X);
            VertexData.Add(pt.Y);
            VertexData.Add(pt.Z);

            VertexData.Add(n.X);
            VertexData.Add(n.Y);
            VertexData.Add(n.Z);

            if ((double.IsNaN(pt.X) || double.IsNaN(pt.Y) || double.IsNaN(pt.Z)) && Eps.messages)
            {
                Console.WriteLine("NaN in geom!");
            }

            if ((double.IsNaN(n.X) || double.IsNaN(n.Y) || double.IsNaN(n.Z)) && Eps.messages)
            {
                Console.WriteLine("NaN in geom!");
            }

            NumPoints += 1;
        }

        public void AddFace(Vec3 a, Vec3 b, Vec3 c, uint pId)
        {
            if (!ComputeSafeNormal(a, b, c, out var normal, Eps.toleranceAddFace))
            {
                if (Eps.messages)
                {
                    Console.WriteLine("zero triangle, AddFace(vec, vec, vec)");
                }

                return;
            }

            AddPoint(a, normal);
            AddPoint(b, normal);
            AddPoint(c, normal);

            AddFace(NumPoints - 3, NumPoints - 2, NumPoints - 1, pId);
        }

        public void AddFace(uint a, uint b, uint c, uint pId)
        {
            IndexData.Add(a);
            IndexData.Add(b);
            IndexData.Add(c);
            PlaneData.Add(pId);

            if (!ComputeSafeNormal(GetPoint(a), GetPoint(b), GetPoint(c), out _, Eps.toleranceAddFace)
                && Eps.messages)
            {
                Console.WriteLine("zero triangle, AddFace(int, int, int)");
            }

            NumFaces++;
        }

        public Face GetFace(int index)
        {
            return new Face(
                IndexData[index * 3 + 0],
                IndexData[index * 3 + 1],
                IndexData[index * 3 + 2],
                PlaneData[index]);
        }

        public AABB GetFaceBox(int index)
        {
            var aabb = new AABB
            {
                Index = (uint)index
            };

            var a = GetPoint(IndexData[index * 3 + 0]);
            var b = GetPoint(IndexData[index * 3 + 1]);
            var c = GetPoint(IndexData[index * 3 + 2]);

            aabb.Min = Vec3.Min(a, aabb.Min);
            aabb.Min = Vec3.Min(b, aabb.Min);
            aabb.Min = Vec3.Min(c, aabb.Min);

            aabb.Max = Vec3.Max(a, aabb.Max);
            aabb.Max = Vec3.Max(b, aabb.Max);
            aabb.Max = Vec3.Max(c, aabb.Max);

            aabb.Center = (aabb.Max + aabb.Min) / 2.0;
            return aabb;
        }

        public Vec3 GetPoint(uint index)
        {
            var o = (int)index * VERTEX_FORMAT_SIZE_FLOATS;
            return new Vec3(
                VertexData[o + 0],
                VertexData[o + 1],
                VertexData[o + 2]);
        }

        public void GetCenterExtents(out Vec3 center, out Vec3 extents)
        {
            const double DblMinPositive = 2.2250738585072014E-308;
            var min = new Vec3(double.MaxValue, double.MaxValue, double.MaxValue);
            var max = new Vec3(DblMinPositive, DblMinPositive, DblMinPositive);

            for (var i = 0u; i < NumPoints; i++)
            {
                var pt = GetPoint(i);
                min = Vec3.Min(min, pt);
                max = Vec3.Max(max, pt);
            }

            extents = max - min;
            center = min + extents / 2.0;
        }

        public Geometry Normalize(Vec3 center, Vec3 extents)
        {
            var newGeom = new Geometry();
            var scale = Math.Max(extents.X, Math.Max(extents.Y, extents.Z)) / 10.0;

            for (var i = 0u; i < NumFaces; i++)
            {
                var face = GetFace((int)i);
                var pa = GetPoint(face.I0);
                var pb = GetPoint(face.I1);
                var pc = GetPoint(face.I2);

                _ = (pa - center) / scale;
                _ = (pb - center) / scale;
                _ = (pc - center) / scale;

                // Kept equivalent to C++ implementation.
                newGeom.AddFace(pa, pb, pc, face.PId);
            }

            return newGeom;
        }

        public Geometry DeNormalize(Vec3 center, Vec3 extents)
        {
            var newGeom = new Geometry();
            var scale = Math.Max(extents.X, Math.Max(extents.Y, extents.Z)) / 10.0;

            for (var i = 0u; i < NumFaces; i++)
            {
                var face = GetFace((int)i);
                var pa = GetPoint(face.I0);
                var pb = GetPoint(face.I1);
                var pc = GetPoint(face.I2);

                var a = pa * scale + center;
                var b = pb * scale + center;
                var c = pc * scale + center;

                newGeom.AddFace(a, b, c, face.PId);
            }

            return newGeom;
        }

        public bool IsEmpty()
        {
            return VertexData.Count == 0;
        }

        public double Volume(Matrix4x4? trans = null)
        {
            var transform = trans ?? Matrix4x4.Identity;
            var totalVolume = 0.0;

            for (var i = 0u; i < NumFaces; i++)
            {
                var f = GetFace((int)i);

                var a = TransformPoint(transform, GetPoint(f.I0));
                var b = TransformPoint(transform, GetPoint(f.I1));
                var c = TransformPoint(transform, GetPoint(f.I2));

                if (ComputeSafeNormal(a, b, c, out var norm, Eps.EPS_SMALL))
                {
                    var area = AreaOfTriangle(a, b, c);
                    var height = Vec3.Dot(norm, a);
                    var tetraVolume = area * height / 3.0;
                    totalVolume += tetraVolume;
                }
            }

            return totalVolume;
        }

        private static Vec3 TransformPoint(Matrix4x4 transform, Vec3 point)
        {
            var v = Vector4.Transform(
                new Vector4((float)point.X, (float)point.Y, (float)point.Z, 1f),
                transform);
            return new Vec3(v.X, v.Y, v.Z);
        }

        private static double AreaOfTriangle(Vec3 a, Vec3 b, Vec3 c)
        {
            var ab = b - a;
            var ac = c - a;
            var norm = Vec3.Cross(ab, ac);
            return norm.Length() / 2.0;
        }

        private static bool ComputeSafeNormal(Vec3 a, Vec3 b, Vec3 c, out Vec3 normal, double eps)
        {
            var v12 = b - a;
            var v13 = c - a;
            var norm = Vec3.Cross(v12, v13);
            var len = norm.Length();

            if (len <= eps)
            {
                normal = new Vec3(0, 0, 0);
                return false;
            }

            normal = norm / len;
            return true;
        }
    }
}
