using System;

namespace BazanCDE.Parsing.Operations.BooleanUtils
{
    public static partial class Utils
    {
        public static bool IntersectRayTriangle(
            Vec3 origin,
            Vec3 end,
            Vec3 v0,
            Vec3 v1,
            Vec3 v2,
            out Vec3 hitPosition,
            out double t,
            out double dPlane,
            bool infiniteLength = false)
        {
            _ = infiniteLength; // kept for parity with C++ signature

            hitPosition = default;
            t = 0.0;
            dPlane = 0.0;

            /*
                Compute the direction of the ray from origin to end.
            */
            var dir = end - origin;
            var dirLength = dir.Length();
            if (dirLength <= Eps.EPS_NONZERO)
            {
                return false;
            }

            /*
                Compute the normal to the plane defined by v0, v1 and v2.
                This normal vector is not normalised.
            */
            var v0v1 = v1 - v0;
            var v0v2 = v2 - v0;
            var n = Vec3.Cross(v0v1, v0v2);

            var nLength = n.Length();
            if (nLength <= Eps.EPS_NONZERO)
            {
                return false;
            }

            /*
                Step 0
                ------
                Check if the ray is parallel to the plane.
            */
            var nDotRayDirection = Vec3.Dot(n, dir);
            if (Math.Abs(nDotRayDirection) < Eps.toleranceParallelTight)
            {
                return false;
            }

            /*
                Step 1
                ------
                Find the point of intersection p of the ray with the plane.
            */
            var d = -Vec3.Dot(n, v0);
            var dOrigin = Vec3.Dot(n, origin);

            t = -(dOrigin + d) / nDotRayDirection;

            // if triangle is behind the ray.
            if (t < -Eps._TOLERANCE_BACK_DEVIATION_DISTANCE)
            {
                return false;
            }

            var p = origin + t * dir;
            var nNorm = n / nLength;
            dPlane = Vec3.Dot(t * dir, nNorm);

            /*
                Step 2
                ------
                Inside-outside test.
            */
            // edge 0
            var edge0 = v1 - v0;
            var v0p = p - v0;
            var c = Vec3.Cross(edge0, v0p);
            var valdot = Vec3.Dot(n, c);
            if (valdot < -Eps._TOLERANCE_INSIDE_OUTSIDE_PERIMETER)
            {
                return false;
            }

            // edge 1
            var edge1 = v2 - v1;
            var v1p = p - v1;
            c = Vec3.Cross(edge1, v1p);
            valdot = Vec3.Dot(n, c);
            if (valdot < -Eps._TOLERANCE_INSIDE_OUTSIDE_PERIMETER)
            {
                return false;
            }

            // edge 2
            var edge2 = v0 - v2;
            var v2p = p - v2;
            c = Vec3.Cross(edge2, v2p);
            valdot = Vec3.Dot(n, c);
            if (valdot < -Eps._TOLERANCE_INSIDE_OUTSIDE_PERIMETER)
            {
                return false;
            }

            hitPosition = p;
            return true; // The ray hits the inside of the triangle.
        }
    }
}
