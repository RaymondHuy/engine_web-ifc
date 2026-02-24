using BazanCDE.Parsing.Geometry.Operations.BooleanUtils;
namespace BazanCDE.Parsing.Utilities
{
    public readonly record struct Vec2(double X, double Y)
    {
        public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
        public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
        public static Vec2 operator *(Vec2 a, double s) => new(a.X * s, a.Y * s);
        public static Vec2 operator *(double s, Vec2 a) => new(a.X * s, a.Y * s);
        public static Vec2 operator /(Vec2 a, double s) => new(a.X / s, a.Y / s);

        public double Length() => Math.Sqrt(X * X + Y * Y);

        public static double Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y;

        public static double Distance(Vec2 a, Vec2 b) => (a - b).Length();

        public static Vec2 Normalize(Vec2 v)
        {
            var len = v.Length();
            if (len <= Eps.EPS_NONZERO)
            {
                return new Vec2(0, 0);
            }

            return v / len;
        }
    }
}
