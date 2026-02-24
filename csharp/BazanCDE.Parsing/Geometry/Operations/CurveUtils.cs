using BazanCDE.Parsing.Geometry.Operations.BimGeometry;
using BazanCDE.Parsing.Geometry.Representation;
using BazanCDE.Parsing.Utilities;

namespace BazanCDE.Parsing.Geometry.Operations
{
    public static partial class CurveUtils
    {
        private static readonly double[] Identity3 =
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };

        public static bool isConvexOrColinear(Vec2 a, Vec2 b, Vec2 c)
        {
            return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X) >= 0;
        }

        public static IfcCurve Build3DArc3Pt(Vec3 p1, Vec3 p2, Vec3 p3, ushort circleSegments, double epsMinSize)
        {
            var v1 = p2 - p1;
            var v2 = p3 - p1;

            if (Vec3.Cross(v1, v2).Length() < epsMinSize)
            {
                return new IfcCurve();
            }

            var cx = p2.X - p1.X;
            var cy = p2.Y - p1.Y;
            var cz = p2.Z - p1.Z;
            var bx = p3.X - p1.X;
            var by = p3.Y - p1.Y;
            var bz = p3.Z - p1.Z;
            var b2 = p1.X * p1.X - p3.X * p3.X + p1.Y * p1.Y - p3.Y * p3.Y + p1.Z * p1.Z - p3.Z * p3.Z;
            var c2 = p1.X * p1.X - p2.X * p2.X + p1.Y * p1.Y - p2.Y * p2.Y + p1.Z * p1.Z - p2.Z * p2.Z;

            var cByz = cy * bz - cz * by;
            var cBxz = cx * bz - cz * bx;
            var cBxy = cx * by - cy * bx;
            var zz1 = -(bz - cz * bx / cx) / (by - cy * bx / cx);
            var z01 = -(b2 - bx / cx * c2) / (2 * (by - cy * bx / cx));
            var zz2 = -(zz1 * cy + cz) / cx;
            var z02 = -(2 * z01 * cy + c2) / (2 * cx);

            var dz = -((z02 - p1.X) * cByz - (z01 - p1.Y) * cBxz - p1.Z * cBxy) / (zz2 * cByz - zz1 * cBxz + cBxy);
            var dx = zz2 * dz + z02;
            var dy = zz1 * dz + z01;

            var center = new Vec3(dx, dy, dz);
            var radius = (center - p1).Length();

            var pointList = new List<Vec3> { p1, p2, p3 };
            while (pointList.Count < circleSegments)
            {
                var tempPointList = new List<Vec3>();
                for (var j = 0; j < pointList.Count - 1; j++)
                {
                    var pt = (pointList[j] + pointList[j + 1]) / 2.0;
                    var vc = Vec3.Normalize(pt - center);
                    pt = center + vc * radius;
                    tempPointList.Add(pointList[j]);
                    tempPointList.Add(pt);
                }

                tempPointList.Add(pointList[pointList.Count - 1]);
                pointList = tempPointList;
            }

            var curve = new IfcCurve();
            for (var j = 0; j < pointList.Count; j++)
            {
                curve.Add(pointList[j]);
            }

            return curve;
        }

        public static IfcCurve BuildArc3Pt(Vec2 p1, Vec2 p2, Vec2 p3, ushort circleSegments)
        {
            var f1 = p1.X * p1.X - p2.X * p2.X + p1.Y * p1.Y - p2.Y * p2.Y;
            var f2 = p1.X * p1.X - p3.X * p3.X + p1.Y * p1.Y - p3.Y * p3.Y;
            var v = 2 * (p1.X - p2.X) * (p1.Y - p3.Y) - 2 * (p1.X - p3.X) * (p1.Y - p2.Y);

            var cenX = ((p1.Y - p3.Y) * f1 - (p1.Y - p2.Y) * f2) / v;
            var den1 = 2 * (p1.Y - p3.Y);
            var den2 = 2 * (p1.Y - p2.Y);
            var cenYa = (f2 - 2 * cenX * (p1.X - p3.X)) / den1;
            var cenYb = (f1 - 2 * cenX * (p1.X - p2.X)) / den2;
            var cenY = Math.Abs(den1) > Math.Abs(den2) ? cenYa : cenYb;

            if (double.IsNaN(cenY) || double.IsInfinity(cenY))
            {
                if (!double.IsNaN(cenYa) && !double.IsInfinity(cenYa))
                {
                    cenY = cenYa;
                }
                else if (!double.IsNaN(cenYb) && !double.IsInfinity(cenYb))
                {
                    cenY = cenYb;
                }
            }

            var center = new Vec2(cenX, cenY);
            var radius = Math.Sqrt((cenX - p1.X) * (cenX - p1.X) + (cenY - p1.Y) * (cenY - p1.Y));

            var pointList = new List<Vec2> { p1, p2, p3 };
            while (pointList.Count < circleSegments)
            {
                var tempPointList = new List<Vec2>();
                for (var j = 0; j < pointList.Count - 1; j++)
                {
                    var pt = pointList[j] + pointList[j + 1];
                    pt = new Vec2(pt.X / 2.0, pt.Y / 2.0);
                    var vc = Vec2.Normalize(pt - center);
                    pt = center + vc * radius;
                    tempPointList.Add(pointList[j]);
                    tempPointList.Add(pt);
                }

                tempPointList.Add(pointList[pointList.Count - 1]);
                pointList = tempPointList;
            }

            var curve = new IfcCurve();
            for (var j = 0; j < pointList.Count; j++)
            {
                curve.Add(pointList[j]);
            }

            return curve;
        }

        public static Vec3 InterpolateRationalBSplineCurveWithKnots(
            double t,
            int degree,
            IReadOnlyList<Vec3> points,
            IReadOnlyList<double> knots,
            IReadOnlyList<double> weights)
        {
            var domainLow = degree;
            var domainHigh = knots.Count - 1 - degree;

            var low = knots[domainLow];
            var high = knots[domainHigh];
            var tPrime = t * (high - low) + low;

            if (tPrime < low || tPrime > high)
            {
                return new Vec3(0, 0, 0);
            }

            var s = 0;
            for (var i = domainLow; i < domainHigh; i++)
            {
                if (knots[i] <= tPrime && tPrime < knots[i + 1])
                {
                    s = i;
                    break;
                }
            }

            if (s == 0)
            {
                s = domainHigh - 1;
            }

            var homogeneousPoints = new List<Vec4>(points.Count);
            for (var i = 0; i < points.Count; i++)
            {
                var p = points[i];
                homogeneousPoints.Add(new Vec4(p.X * weights[i], p.Y * weights[i], p.Z * weights[i], weights[i]));
            }

            for (var l = 1; l <= degree + 1; l++)
            {
                for (var i = s; i > s - degree - 1 + l; i--)
                {
                    var alpha = (tPrime - knots[i]) / (knots[i + degree + 1 - l] - knots[i]);
                    if (double.IsNaN(alpha) || double.IsInfinity(alpha))
                    {
                        alpha = 1.0;
                    }

                    var x = (1 - alpha) * homogeneousPoints[i - 1].X + alpha * homogeneousPoints[i].X;
                    var y = (1 - alpha) * homogeneousPoints[i - 1].Y + alpha * homogeneousPoints[i].Y;
                    var z = (1 - alpha) * homogeneousPoints[i - 1].Z + alpha * homogeneousPoints[i].Z;
                    var w = (1 - alpha) * homogeneousPoints[i - 1].W + alpha * homogeneousPoints[i].W;
                    homogeneousPoints[i] = new Vec4(x, y, z, w);
                }
            }

            var hs = homogeneousPoints[s];
            return new Vec3(hs.X / hs.W, hs.Y / hs.W, hs.Z / hs.W);
        }

        public static Vec2 InterpolateRationalBSplineCurveWithKnots(
            double t,
            int degree,
            IReadOnlyList<Vec2> points,
            IReadOnlyList<double> knots,
            IReadOnlyList<double> weights)
        {
            var domainLow = degree;
            var domainHigh = knots.Count - 1 - degree;

            var low = knots[domainLow];
            var high = knots[domainHigh];
            var tPrime = t * (high - low) + low;

            if (tPrime < low || tPrime > high)
            {
                return new Vec2(0, 0);
            }

            var s = 0;
            for (var i = domainLow; i < domainHigh; i++)
            {
                if (knots[i] <= tPrime && tPrime < knots[i + 1])
                {
                    s = i;
                    break;
                }
            }

            var homogeneousPoints = new List<Vec3>(points.Count);
            for (var i = 0; i < points.Count; i++)
            {
                var p = points[i];
                homogeneousPoints.Add(new Vec3(p.X * weights[i], p.Y * weights[i], weights[i]));
            }

            for (var l = 1; l <= degree + 1; l++)
            {
                for (var i = s; i > s - degree - 1 + l; i--)
                {
                    var alpha = (tPrime - knots[i]) / (knots[i + degree + 1 - l] - knots[i]);

                    var x = (1 - alpha) * homogeneousPoints[i - 1].X + alpha * homogeneousPoints[i].X;
                    var y = (1 - alpha) * homogeneousPoints[i - 1].Y + alpha * homogeneousPoints[i].Y;
                    var w = (1 - alpha) * homogeneousPoints[i - 1].Z + alpha * homogeneousPoints[i].Z;
                    homogeneousPoints[i] = new Vec3(x, y, w);
                }
            }

            var hs = homogeneousPoints[s];
            return new Vec2(hs.X / hs.Z, hs.Y / hs.Z);
        }

        public static List<Vec3> GetRationalBSplineCurveWithKnots(
            int degree,
            IReadOnlyList<Vec3> points,
            IReadOnlyList<double> knots,
            IReadOnlyList<double> weights,
            double numCurvePoints)
        {
            var c = new List<Vec3>();
            var step = 1.0 / numCurvePoints;
            for (var i = 0.0; i < 1.0; i += step)
            {
                c.Add(InterpolateRationalBSplineCurveWithKnots(i, degree, points, knots, weights));
            }

            return c;
        }

        public static List<Vec2> GetRationalBSplineCurveWithKnots(
            int degree,
            IReadOnlyList<Vec2> points,
            IReadOnlyList<double> knots,
            IReadOnlyList<double> weights)
        {
            var c = new List<Vec2>();
            for (var i = 0.0; i < 1.0; i += 0.05)
            {
                c.Add(InterpolateRationalBSplineCurveWithKnots(i, degree, points, knots, weights));
            }

            return c;
        }

        public static bool IsCurveConvex(IfcCurve curve)
        {
            for (var i = 2; i < curve.points.Count; i++)
            {
                var a = curve.points[i - 2];
                var b = curve.points[i - 1];
                var c = curve.points[i];

                if (!isConvexOrColinear(new Vec2(a.X, a.Y), new Vec2(b.X, b.Y), new Vec2(c.X, c.Y)))
                {
                    return false;
                }
            }

            return true;
        }

        public static bool MatrixFlipsTriangles(double[] mat)
        {
            return Utils.MatrixFlipsTriangles(mat);
        }

        public static IfcCurve GetEllipseCurve(
            float radiusX,
            float radiusY,
            int numSegments,
            double[]? placement = null,
            double startRad = 0,
            double endRad = IfcRepresentationConstants.CONST_PI * 2,
            bool swap = true,
            bool normalToCenterEnding = false)
        {
            var temp = Utils.GetEllipseCurve(radiusX, radiusY, numSegments, placement, startRad, endRad, swap, normalToCenterEnding);
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetCircleCurve(float radius, int numSegments, double[]? placement = null)
        {
            return GetEllipseCurve(radius, radius, numSegments, placement);
        }

        public static IfcCurve GetRectangleCurve(
            double xdim,
            double ydim,
            double[]? placement = null,
            int numSegments = 12,
            double radius = 0)
        {
            var temp = Utils.GetRectangleCurve(xdim, ydim, ToPlacement4(placement), numSegments, radius);
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetIShapedCurve(
            double width,
            double depth,
            double webThickness,
            double flangeThickness,
            bool hasFillet,
            double filletRadius,
            double[]? placement = null)
        {
            var temp = Utils.GetIShapedCurve(width, depth, webThickness, flangeThickness, hasFillet, filletRadius, ToPlacement4(placement));
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetUShapedCurve(
            double depth,
            double flangeWidth,
            double webThickness,
            double flangeThickness,
            double filletRadius,
            double edgeRadius,
            double flangeSlope,
            double[]? placement = null)
        {
            var temp = Utils.GetUShapedCurve(
                depth,
                flangeWidth,
                webThickness,
                flangeThickness,
                filletRadius,
                edgeRadius,
                flangeSlope,
                ToPlacement4(placement));
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetLShapedCurve(
            double width,
            double depth,
            double thickness,
            bool hasFillet,
            double filletRadius,
            double edgeRadius,
            double legSlope,
            int numSegments,
            double[]? placement = null)
        {
            var temp = Utils.GetLShapedCurve(
                width,
                depth,
                thickness,
                hasFillet,
                filletRadius,
                edgeRadius,
                legSlope,
                numSegments,
                ToPlacement4(placement));
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetTShapedCurve(
            double width,
            double depth,
            double thickness,
            bool hasFillet,
            double filletRadius,
            double edgeRadius,
            double legSlope,
            double[]? placement = null)
        {
            var temp = Utils.GetTShapedCurve(width, depth, thickness, hasFillet, filletRadius, edgeRadius, legSlope, ToPlacement4(placement));
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetCShapedCurve(
            double width,
            double depth,
            double girth,
            double thickness,
            bool hasFillet,
            double filletRadius,
            double[]? placement = null)
        {
            var temp = Utils.GetCShapedCurve(width, depth, girth, thickness, hasFillet, filletRadius, ToPlacement4(placement));
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetZShapedCurve(
            double depth,
            double flangeWidth,
            double webThickness,
            double flangeThickness,
            double filletRadius,
            double edgeRadius,
            double[]? placement = null)
        {
            var temp = Utils.GetZShapedCurve(depth, flangeWidth, webThickness, flangeThickness, filletRadius, edgeRadius, ToPlacement4(placement));
            return ToIfcCurve(temp);
        }

        public static IfcCurve GetTrapeziumCurve(
            double bottomXDim,
            double topXDim,
            double yDim,
            double topXOffset,
            double[]? placement = null)
        {
            var temp = Utils.GetTrapeziumCurve(bottomXDim, topXDim, yDim, topXOffset, ToPlacement4(placement));
            return ToIfcCurve(temp);
        }

        public static IfcCurve BuildArc(
            double scale,
            Vec3 pos,
            Vec3 axis,
            double angleRad,
            ushort circleSegments)
        {
            _ = scale;

            var curve = new IfcCurve();

            var pDotA = Vec3.Dot(axis, pos);
            var pProjA = pDotA * axis;
            var right = (pos - pProjA) * -1.0;

            if (right.Length() == 0)
            {
                right = new Vec3(Epsilons.EPS_BIG2, 0, 0);
                var up = Vec3.Cross(axis, right);
                var curve2D = GetEllipseCurve(1, 1, circleSegments, Identity3, 0, angleRad, true, true);

                foreach (var pt2D in curve2D.points)
                {
                    var pt3D = pos + pt2D.X * right + pt2D.Y * up;
                    curve.Add(pt3D);
                }
            }
            else
            {
                var up = Vec3.Cross(axis, right);
                var curve2D = GetEllipseCurve(1, 1, circleSegments, Identity3, 0, angleRad, true);

                foreach (var pt2D in curve2D.points)
                {
                    var pt3D = pos + pt2D.X * right + pt2D.Y * up;
                    curve.Add(pt3D);
                }
            }

            return curve;
        }

        private static IfcCurve ToIfcCurve(Curve temp)
        {
            var c = new IfcCurve();
            c.points.AddRange(temp.points);
            return c;
        }

        private static double[] ToPlacement4(double[]? placement3)
        {
            var p3 = placement3 ?? Identity3;
            if (p3.Length < 9)
            {
                p3 = Identity3;
            }

            return
            [
                p3[0], p3[1], 0, 0,
                p3[3], p3[4], 0, 0,
                0, 0, 1, 0,
                p3[6], p3[7], 0, 1
            ];
        }
    }
}
