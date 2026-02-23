namespace BazanCDE.Parsing.Operations.BooleanUtils
{
    public readonly record struct Vec4(double X, double Y, double Z, double W)
    {
        public Vec3 XYZ => new(X, Y, Z);
    }
}
