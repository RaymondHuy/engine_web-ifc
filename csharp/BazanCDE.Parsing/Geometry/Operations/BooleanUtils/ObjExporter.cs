using System.IO;
using System.Numerics;
using System.Text;

namespace BazanCDE.Parsing.Operations.BooleanUtils
{
    public static partial class Utils
    {
        public static string ToObj(Geometry geom, ref ulong offset, Matrix4x4? transform = null, double inputScale = 1.0)
        {
            var obj = new StringBuilder();
            var mat = transform ?? Matrix4x4.Identity;
            var scale = inputScale;

            for (var i = 0u; i < geom.NumPoints; i++)
            {
                var t = TransformPoint(mat, geom.GetPoint(i));
                obj.Append("v ")
                    .Append(t.X * scale).Append(' ')
                    .Append(t.Y * scale).Append(' ')
                    .Append(t.Z * scale).Append('\n');
            }

            for (var i = 0u; i < geom.NumFaces; i++)
            {
                var f = geom.GetFace((int)i);
                obj.Append("f ")
                    .Append(f.I0 + 1 + offset).Append("// ")
                    .Append(f.I1 + 1 + offset).Append("// ")
                    .Append(f.I2 + 1 + offset).Append("//\n");
            }

            offset += geom.NumPoints;
            return obj.ToString();
        }

        public static void DumpGeometry(Geometry geom, string filename)
        {
            ulong offset = 0;
            File.WriteAllText(filename, ToObj(geom, ref offset));
        }

        private static Vec3 TransformPoint(Matrix4x4 transform, Vec3 point)
        {
            var transformed = Vector4.Transform(
                new Vector4((float)point.X, (float)point.Y, (float)point.Z, 1f),
                transform);
            return new Vec3(transformed.X, transformed.Y, transformed.Z);
        }
    }
}
