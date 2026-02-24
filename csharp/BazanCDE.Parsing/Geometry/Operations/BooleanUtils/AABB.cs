using BazanCDE.Parsing.Utilities;
namespace BazanCDE.Parsing.Geometry.Operations.BooleanUtils
{
    public class AABB
    {
        public uint Index;
        public Vec3 Min = new(double.MaxValue, double.MaxValue, double.MaxValue);
        public Vec3 Max = new(-double.MaxValue, -double.MaxValue, -double.MaxValue);
        public Vec3 Center = new(0, 0, 0);

        public bool Intersects(AABB other)
        {
            var eps = Eps.toleranceAABB;
            return (Max.X + eps >= other.Min.X && other.Max.X + eps >= Min.X
                && Max.Y + eps >= other.Min.Y && other.Max.Y + eps >= Min.Y
                && Max.Z + eps >= other.Min.Z && other.Max.Z + eps >= Min.Z);
        }

        public bool Contains(Vec3 pos)
        {
            var eps = Eps.toleranceAABB;
            return pos.X + eps >= Min.X && pos.X - eps <= Max.X
                && pos.Y + eps >= Min.Y && pos.Y - eps <= Max.Y
                && pos.Z + eps >= Min.Z && pos.Z - eps <= Max.Z;
        }

        public void Merge(AABB other)
        {
            Min = Vec3.Min(Min, other.Min);
            Max = Vec3.Max(Max, other.Max);
        }

        public void Merge(Vec3 other)
        {
            Min = Vec3.Min(Min, other);
            Max = Vec3.Max(Max, other);
        }

        // https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
        public bool Intersect(Vec3 origin, Vec3 dir)
        {
            // r.dir is unit direction vector of ray
            var dirfracX = 1.0 / dir.X;
            var dirfracY = 1.0 / dir.Y;
            var dirfracZ = 1.0 / dir.Z;

            // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
            // r.org is origin of ray
            var t1 = (Min.X - origin.X) * dirfracX;
            var t2 = (Max.X - origin.X) * dirfracX;
            var t3 = (Min.Y - origin.Y) * dirfracY;
            var t4 = (Max.Y - origin.Y) * dirfracY;
            var t5 = (Min.Z - origin.Z) * dirfracZ;
            var t6 = (Max.Z - origin.Z) * dirfracZ;

            var tmin = Math.Max(
                Math.Max(Math.Min(t1, t2), Math.Min(t3, t4)),
                Math.Min(t5, t6));
            var tmax = Math.Min(
                Math.Min(Math.Max(t1, t2), Math.Max(t3, t4)),
                Math.Max(t5, t6));

            // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
            if (tmax < -Eps._TOLERANCE_BOUNDING_BOX)
            {
                return false;
            }

            // if tmin > tmax, ray doesn't intersect AABB
            if (tmin > tmax + Eps._TOLERANCE_BOUNDING_BOX)
            {
                return false;
            }

            return true;
        }
    }
}
