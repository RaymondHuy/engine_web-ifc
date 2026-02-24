using BazanCDE.Parsing.Utilities;
namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class AABB
    {
        private const double EPS_BIG = 1.0E-04;

        public uint index;
        public Vec3 min = new(double.MaxValue, double.MaxValue, double.MaxValue);
        public Vec3 max = new(-double.MaxValue, -double.MaxValue, -double.MaxValue);
        public Vec3 center = new(0, 0, 0);

        public bool intersects(AABB other)
        {
            var eps = EPS_BIG;
            return max.X + eps >= other.min.X && other.max.X + eps >= min.X
                && max.Y + eps >= other.min.Y && other.max.Y + eps >= min.Y
                && max.Z + eps >= other.min.Z && other.max.Z + eps >= min.Z;
        }

        public bool contains(Vec3 pos)
        {
            var eps = EPS_BIG;
            return pos.X + eps >= min.X && pos.X - eps <= max.X
                && pos.Y + eps >= min.Y && pos.Y - eps <= max.Y
                && pos.Z + eps >= min.Z && pos.Z - eps <= max.Z;
        }

        public void merge(AABB other)
        {
            min = Vec3.Min(min, other.min);
            max = Vec3.Max(max, other.max);
        }

        public void merge(Vec3 other)
        {
            min = Vec3.Min(min, other);
            max = Vec3.Max(max, other);
        }

        public bool Intersect(Vec3 origin, Vec3 dir)
        {
            var dirfracX = 1.0 / dir.X;
            var dirfracY = 1.0 / dir.Y;
            var dirfracZ = 1.0 / dir.Z;

            var t1 = (min.X - origin.X) * dirfracX;
            var t2 = (max.X - origin.X) * dirfracX;
            var t3 = (min.Y - origin.Y) * dirfracY;
            var t4 = (max.Y - origin.Y) * dirfracY;
            var t5 = (min.Z - origin.Z) * dirfracZ;
            var t6 = (max.Z - origin.Z) * dirfracZ;

            var tmin = Math.Max(
                Math.Max(Math.Min(t1, t2), Math.Min(t3, t4)),
                Math.Min(t5, t6));
            var tmax = Math.Min(
                Math.Min(Math.Max(t1, t2), Math.Max(t3, t4)),
                Math.Max(t5, t6));

            if (tmax < -EPS_BIG)
            {
                return false;
            }

            if (tmin > tmax + EPS_BIG)
            {
                return false;
            }

            return true;
        }

        public void SetValues(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
        {
            min = new Vec3(minX, minY, minZ);
            max = new Vec3(maxX, maxY, maxZ);
            center = new Vec3(
                (minX + maxX) / 2.0,
                (minY + maxY) / 2.0,
                (minZ + maxZ) / 2.0);
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();

            buffers.AddPoint(new Vec3(max.X, max.Y, min.Z));
            buffers.AddPoint(new Vec3(max.X, min.Y, min.Z));
            buffers.AddPoint(new Vec3(max.X, min.Y, max.Z));
            buffers.AddPoint(new Vec3(max.X, max.Y, max.Z));
            buffers.AddPoint(new Vec3(min.X, max.Y, min.Z));
            buffers.AddPoint(new Vec3(min.X, min.Y, min.Z));
            buffers.AddPoint(new Vec3(min.X, min.Y, max.Z));
            buffers.AddPoint(new Vec3(min.X, max.Y, max.Z));

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
}
