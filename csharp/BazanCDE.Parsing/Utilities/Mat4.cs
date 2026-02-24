namespace BazanCDE.Parsing.Utilities
{
    public readonly record struct Mat4(Vec4 Row1, Vec4 Row2, Vec4 Row3, Vec4 Row4)
    {
        public static Mat4 Identity => new(
            new Vec4(1, 0, 0, 0),
            new Vec4(0, 1, 0, 0),
            new Vec4(0, 0, 1, 0),
            new Vec4(0, 0, 0, 1)
        );
        
        public static Mat4 operator *(Mat4 a, Mat4 b)
        {
            return new Mat4(
                MultiplyRow(a.Row1, b),
                MultiplyRow(a.Row2, b),
                MultiplyRow(a.Row3, b),
                MultiplyRow(a.Row4, b)
            );
        }
        
        private static Vec4 MultiplyRow(Vec4 row, Mat4 b)
        {
            return new Vec4(
                row.X * b.Row1.X + row.Y * b.Row2.X + row.Z * b.Row3.X + row.W * b.Row4.X,
                row.X * b.Row1.Y + row.Y * b.Row2.Y + row.Z * b.Row3.Y + row.W * b.Row4.Y,
                row.X * b.Row1.Z + row.Y * b.Row2.Z + row.Z * b.Row3.Z + row.W * b.Row4.Z,
                row.X * b.Row1.W + row.Y * b.Row2.W + row.Z * b.Row3.W + row.W * b.Row4.W
            );
        }
    }
}
