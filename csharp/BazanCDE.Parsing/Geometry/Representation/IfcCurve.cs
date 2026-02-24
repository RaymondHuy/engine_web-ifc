using BazanCDE.Parsing.Utilities;
using BazanCDE.Parsing.Geometry.Operations.BimGeometry;

namespace BazanCDE.Parsing.Geometry.Representation
{
    public class IfcCurve : Curve
    {
        public readonly List<uint> arcSegments = new();
        public readonly List<string> userData = new();
        public readonly List<ushort> indices = new();
        public Vec3 endTangent = new(0, 0, 0);
        // Stores the precise analytic tangent for the start of each segment, in case of IfcCurveSegment.
        public readonly List<Vec3> segmentStartTangents = new();

        protected const double EPS_TINY = 1e-9;

        public enum CurvePlacementMode
        {
            TangentAsZAxis,
            GlobalZAxis
        }

        public Vec2 Get2d(ulong i)
        {
            var ret = points[(int)i];
            return new Vec2(ret.X, ret.Y);
        }

        public Vec2 Get2d(int i)
        {
            return Get2d((ulong)i);
        }

        public Vec3 Get3d(ulong i)
        {
            return points[(int)i];
        }

        public Vec3 Get3d(int i)
        {
            return Get3d((ulong)i);
        }

        /// <summary>
        /// Get a transformation matrix at a specified distance along the curve.
        /// Z axis is aligned with the curve tangent, or with global z axis, depending on mode.
        /// Returned matrix is column-major 4x4 (length 16).
        /// </summary>
        public double[] getPlacementAtDistance(double distance, CurvePlacementMode mode)
        {
            var globalZ = new Vec3(0.0, 0.0, 1.0);
            const double eps = 1e-6;

            if (points.Count == 0)
            {
                return
                [
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1
                ];
            }

            var pos = points.Count == 1 ? points[0] : new Vec3(0, 0, 0);
            var tan = new Vec3(1.0, 0.0, 0.0);

            // Locate position and tangent.
            var totalDistance = 0.0;
            var found = false;
            for (var i = 0; i < points.Count - 1; i++)
            {
                var segLength = (points[i] - points[i + 1]).Length();
                if (segLength < eps)
                {
                    continue;
                }

                totalDistance += segLength;
                if (totalDistance >= distance - eps)
                {
                    var segStartLength = totalDistance - segLength;
                    var factor = Math.Clamp((distance - segStartLength) / segLength, 0.0, 1.0);
                    pos = points[i] * (1.0 - factor) + points[i + 1] * factor;
                    tan = points[i + 1] - points[i];
                    found = true;
                    break;
                }
            }

            // Clamp to endpoint.
            if (!found && points.Count > 1)
            {
                pos = points[points.Count - 1];
                tan = points[points.Count - 1] - points[points.Count - 2];
            }

            if (tan.Length() > eps)
            {
                tan = Vec3.Normalize(tan);
            }
            else
            {
                tan = new Vec3(1.0, 0.0, 0.0);
            }

            Vec3 vx;
            Vec3 vy;
            Vec3 vz;

            if (mode == CurvePlacementMode.GlobalZAxis)
            {
                // Tangent is local X, local Z is global Z.
                vx = tan;
                vz = globalZ;

                vy = Vec3.Cross(vz, vx);
                if (vy.Length() < eps)
                {
                    vy = new Vec3(0.0, 1.0, 0.0);
                }

                vy = Vec3.Normalize(vy);
                vx = Vec3.Normalize(Vec3.Cross(vy, vz));
            }
            else
            {
                // Frenet-style: tangent is local Z.
                vz = tan;

                vx = Vec3.Cross(globalZ, vz);
                if (vx.Length() < eps)
                {
                    vx = new Vec3(1.0, 0.0, 0.0);
                }

                vx = Vec3.Normalize(vx);
                vy = Vec3.Normalize(Vec3.Cross(vz, vx));
                vx = Vec3.Normalize(Vec3.Cross(vy, vz));
            }

            // Column-major matrix: [vx vy vz pos].
            return
            [
                vx.X, vx.Y, vx.Z, 0.0,
                vy.X, vy.Y, vy.Z, 0.0,
                vz.X, vz.Y, vz.Z, 0.0,
                pos.X, pos.Y, pos.Z, 1.0
            ];
        }
    }
}
