using BazanCDE.Parsing.Utilities;
namespace BazanCDE.Parsing.Geometry.Operations.BooleanUtils
{
    public static partial class Utils
    {
        public struct InsideResult
        {
            public MeshLocation Loc;
            public Vec3 Normal;
        }

        public static InsideResult IsInsideMesh(
            Vec3 pt,
            Vec3 normal,
            Geometry g,
            BVH bvh,
            Vec3? dir = null,
            bool union = false)
        {
            var winding = 0;
            var rayDir = dir ?? new Vec3(1.0, 1.1, 1.4);
            rayDir = rayDir + new Vec3(0.02, 0.01, 0.04); // randomize ray direction

            var result = new InsideResult
            {
                Loc = MeshLocation.BOUNDARY,
                Normal = new Vec3(0, 0, 0)
            };

            var hasResult = bvh.IntersectRay(pt, rayDir, i =>
            {
                var f = g.GetFace((int)i);
                var a = g.GetPoint(f.I0);
                var b = g.GetPoint(f.I1);
                var c = g.GetPoint(f.I2);

                var hasIntersection = IntersectRayTriangle(
                    pt,
                    pt + rayDir,
                    a,
                    b,
                    c,
                    out _,
                    out _,
                    out var dPlane,
                    infiniteLength: true);

                if (!hasIntersection)
                {
                    return false; // continue
                }

                var otherNormal = ComputeNormal(a, b, c); // normalized
                var d = Vec3.Dot(otherNormal, rayDir);
                _ = d;
                var dn = Vec3.Dot(otherNormal, normal);

                if (Math.Abs(dPlane) < Eps._TOLERANCE_PLANE_DEVIATION)
                {
                    if (dn > 1.0 - Eps.toleranceParallel)
                    {
                        // Normals point in same direction => inside boundary.
                        result.Loc = MeshLocation.BOUNDARY;
                        result.Normal = normal;
                        return true;
                    }

                    if (dn < -1.0 + Eps.toleranceParallel)
                    {
                        // Normals point in opposite directions => outside boundary.
                        if (!union)
                        {
                            result.Loc = MeshLocation.OUTSIDE;
                            result.Normal = normal;
                            return true;
                        }

                        result.Loc = MeshLocation.BOUNDARY;
                        result.Normal = normal;
                        return true;
                    }

                    result.Loc = MeshLocation.BOUNDARY;
                    result.Normal = otherNormal;
                    return true;
                }

                winding++;
                return false; // continue
            });

            if (hasResult)
            {
                return result;
            }

            result.Loc = winding % 2 == 1 ? MeshLocation.INSIDE : MeshLocation.OUTSIDE;
            return result;
        }

        private static Vec3 ComputeNormal(Vec3 a, Vec3 b, Vec3 c)
        {
            var n = Vec3.Cross(b - a, c - a);
            var len = n.Length();
            if (len <= Eps.EPS_NONZERO)
            {
                return new Vec3(0, 0, 0);
            }

            return n / len;
        }
    }
}
