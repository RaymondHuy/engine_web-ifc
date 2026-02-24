using BazanCDE.Parsing.Utilities;
namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Plane
    {
        public double distance;
        public Vec3 normal;
        public ulong id;

        public bool IsEqualTo(Vec3 n, double d)
        {
            return Equals(normal, n, Epsilons.toleranceVectorEquality)
                && Equals(distance, d, Epsilons._TOLERANCE_SCALAR_EQUALITY);
        }

        private static bool Equals(Vec3 a, Vec3 b, double eps = 0)
        {
            return Math.Abs(a.X - b.X) <= eps
                && Math.Abs(a.Y - b.Y) <= eps
                && Math.Abs(a.Z - b.Z) <= eps;
        }

        private static bool Equals(double a, double b, double eps = 0)
        {
            return Math.Abs(a - b) <= eps;
        }
    }
}
