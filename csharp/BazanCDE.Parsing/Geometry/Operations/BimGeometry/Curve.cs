using BazanCDE.Parsing.Utilities;

namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Curve
    {
        public readonly List<Vec3> points = new();

        // Fragments -> Issue #89 -> requires "removeCoincident" switch
        public void Add(Vec3 pt, bool removeCoincident = true)
        {
            if (points.Count == 0 || !removeCoincident)
            {
                points.Add(pt);
            }
            else if (!Equals(pt, points[points.Count - 1], Epsilons.EPS_TINY_CURVE))
            {
                points.Add(pt);
            }
        }

        public void Add(Vec2 pt)
        {
            var point = new Vec3(pt.X, pt.Y, 0);
            Add(point);
        }

        public void Invert()
        {
            points.Reverse();
        }

        public bool IsCCW()
        {
            double sum = 0;
            var n = points.Count;

            for (var i = 0; i < n; i++)
            {
                var pt1 = points[(i + n - 1) % n];
                var pt2 = points[i];
                sum += (pt2.X - pt1.X) * (pt2.Y + pt1.Y);
            }

            return sum < 0;
        }

        private static bool Equals(Vec3 a, Vec3 b, double eps = 0)
        {
            return Math.Abs(a.X - b.X) <= eps
                && Math.Abs(a.Y - b.Y) <= eps
                && Math.Abs(a.Z - b.Z) <= eps;
        }
    }
}
