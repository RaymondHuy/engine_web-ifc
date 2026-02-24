using BazanCDE.Parsing.Geometry.Operations.BooleanUtils;
namespace BazanCDE.Parsing.Utilities
{
    public readonly record struct Vec3(double X, double Y, double Z)
    {
        public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator *(Vec3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);
        public static Vec3 operator *(double s, Vec3 a) => new(a.X * s, a.Y * s, a.Z * s);
        public static Vec3 operator /(Vec3 a, double s) => new(a.X / s, a.Y / s, a.Z / s);

        public double this[int axis] => axis switch
        {
            0 => X,
            1 => Y,
            _ => Z
        };

        public double Length() => Math.Sqrt(X * X + Y * Y + Z * Z);

        public static double Dot(Vec3 a, Vec3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        }

        public static Vec3 Min(Vec3 a, Vec3 b)
        {
            return new Vec3(
                Math.Min(a.X, b.X),
                Math.Min(a.Y, b.Y),
                Math.Min(a.Z, b.Z));
        }

        public static Vec3 Max(Vec3 a, Vec3 b)
        {
            return new Vec3(
                Math.Max(a.X, b.X),
                Math.Max(a.Y, b.Y),
                Math.Max(a.Z, b.Z));
        }

        public static Vec3 Normalize(Vec3 v)
        {
            var len = v.Length();
            if (len <= Eps.EPS_NONZERO)
            {
                return new Vec3(0, 0, 0);
            }

            return v / len;
        }

        public static Vec3 Abs(Vec3 v)
        {
            return new Vec3(Math.Abs(v.X), Math.Abs(v.Y), Math.Abs(v.Z));
        }
    }
}
