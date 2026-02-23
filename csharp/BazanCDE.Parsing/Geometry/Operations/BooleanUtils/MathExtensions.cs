using System;
using System.Collections.Generic;
using System.Numerics;

namespace BazanCDE.Parsing.Operations.BooleanUtils
{
    public readonly record struct Mat3d(
        double M11, double M12, double M13,
        double M21, double M22, double M23,
        double M31, double M32, double M33)
    {
        public double Determinant =>
            M11 * (M22 * M33 - M23 * M32)
            - M12 * (M21 * M33 - M23 * M31)
            + M13 * (M21 * M32 - M22 * M31);
    }

    public struct LineLineIsect2D
    {
        public bool isect;
        public Vec2 pt;
        public double dist;
    }

    public struct LineLineIsect
    {
        public double param1;
        public double param2;
        public Vec3 point1;
        public Vec3 point2;
        public double distance;
    }

    public struct PlanePlaneIsectResult
    {
        public Vec3 pos;
        public Vec3 dir;
    }

    public static partial class Utils
    {
        public static bool computeSafeNormal(Vec3 v1, Vec3 v2, Vec3 v3, out Vec3 normal, double eps = 0)
        {
            var v12 = v2 - v1;
            var v13 = v3 - v1;

            _ = Vec3.Cross(v12, v13); // temp in C++ implementation
            var norm = Vec3.Cross(v12, v13);
            var len = norm.Length();

            if (double.IsNaN(len) || len <= eps)
            {
                normal = new Vec3(0, 0, 0);
                return false;
            }

            normal = norm / len;
            return true;
        }

        public static Vec3 computeNormal(Vec3 v1, Vec3 v2, Vec3 v3)
        {
            var v12 = v2 - v1;
            var v13 = v3 - v1;
            var norm = Vec3.Cross(v12, v13);
            return Vec3.Normalize(norm);
        }

        public static bool IsInsideCenterExtents(Vec3 pt, Vec3 center, Vec3 extents)
        {
            var delta = Vec3.Abs(pt - center);
            var offset = delta - extents;
            return offset.X < Eps.toleranceIsInsideCenterExtents
                && offset.Y < Eps.toleranceIsInsideCenterExtents
                && offset.Z < Eps.toleranceIsInsideCenterExtents;
        }

        public static double areaOfTriangle(Vec3 a, Vec3 b, Vec3 c)
        {
            var ab = b - a;
            var ac = c - a;
            var norm = Vec3.Cross(ab, ac);
            return norm.Length() / 2.0;
        }

        public static bool MatrixFlipsTriangles(Matrix4x4 mat)
        {
            return mat.GetDeterminant() < 0;
        }

        public static bool MatrixFlipsTriangles(Mat3d mat)
        {
            return mat.Determinant < 0;
        }

        public static bool equals2d(Vec2 a, Vec2 b, double eps = 0)
        {
            return Math.Abs(a.X - b.X) <= eps && Math.Abs(a.Y - b.Y) <= eps;
        }

        public static bool equals(Vec3 a, Vec3 b, double eps = 0)
        {
            return Math.Abs(a.X - b.X) <= eps
                && Math.Abs(a.Y - b.Y) <= eps
                && Math.Abs(a.Z - b.Z) <= eps;
        }

        public static bool equals(double a, double b, double eps = 0)
        {
            return Math.Abs(a - b) <= eps;
        }

        public static double sign2D(Vec2 p, Vec2 a, Vec2 b)
        {
            return (p.X - b.X) * (a.Y - b.Y) - (a.X - b.X) * (p.Y - b.Y);
        }

        public static double cross2d(Vec2 point1, Vec2 point2)
        {
            return point1.X * point2.Y - point1.Y * point2.X;
        }

        // https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
        public static double signOneZero(double x)
        {
            return (x > 0 ? 1.0 : 0.0) - (x < 0 ? 1.0 : 0.0);
        }

        public static double ComparableAngle(Vec2 p, Vec2 a, Vec2 b)
        {
            var upDown = sign2D(p, a, b) >= 0 ? 1.0 : -1.0;
            var dot = (1 + Vec2.Dot(Vec2.Normalize(a - b), Vec2.Normalize(p - b))) / 2.0;
            return upDown * dot;
        }

        public static bool allEqual(bool b1, bool b2, bool b3, bool b4)
        {
            return b1 == b2 && b1 == b3 && b1 == b4;
        }

        public static LineLineIsect2D doLineSegmentsIntersect(Vec2 p, Vec2 p2, Vec2 q, Vec2 q2, double eps)
        {
            var result = new LineLineIsect2D();

            var r = p2 - p;
            var s = q2 - q;

            var uNumerator = cross2d(q - p, r);
            var denominator = cross2d(r, s);

            if (uNumerator == 0 && denominator == 0)
            {
                // They are collinear.
                if (equals2d(p, q, eps) || equals2d(p, q2, eps) || equals2d(p2, q, eps) || equals2d(p2, q2, eps))
                {
                    result.isect = true;
                    return result;
                }

                // Do they overlap?
                result.isect = 
                    !allEqual(
                        q.X - p.X < 0,
                        q.X - p2.X < 0,
                        q2.X - p.X < 0,
                        q2.X - p2.X < 0)
                   || !allEqual(
                        q.Y - p.Y < 0,
                        q.Y - p2.Y < 0,
                        q2.Y - p.Y < 0,
                        q2.Y - p2.Y < 0);
                return result;
            }

            if (denominator == 0)
            {
                // lines are parallel
                result.isect = false;
                return result;
            }

            var u = uNumerator / denominator;
            var t = cross2d(q - p, s) / denominator;

            result.isect = (t >= -eps) && (t <= 1 + eps) && (u >= -eps) && (u <= 1 + eps);
            result.pt = p + r * t;
            result.dist = Vec2.Distance(p, result.pt);

            return result;
        }

        public static LineLineIsect LineLineIntersection(Vec3 p0, Vec3 p1, Vec3 q0, Vec3 q1)
        {
            var p1mp0 = p1 - p0;
            var q1mq0 = q1 - q0;
            var p0mq0 = p0 - q0;
            var a = Vec3.Dot(p1mp0, p1mp0);
            var b = Vec3.Dot(p1mp0, q1mq0);
            var c = Vec3.Dot(q1mq0, q1mq0);
            var d = Vec3.Dot(p1mp0, p0mq0);
            var e = Vec3.Dot(q1mq0, p0mq0);
            var det = a * c - b * b;
            double s = 0;
            double t = 0;
            double nd;
            double bmd;
            double bte;
            double ctd;
            double bpe;
            double ate;
            double btd;

            const double zero = 0;
            const double one = 1;
            if (det > zero)
            {
                bte = b * e;
                ctd = c * d;
                if (bte <= ctd) // s <= 0
                {
                    s = zero;
                    if (e <= zero) // t <= 0
                    {
                        t = zero;
                        nd = -d;
                        if (nd >= a)
                        {
                            s = one;
                        }
                        else if (nd > zero)
                        {
                            s = nd / a;
                        }
                    }
                    else if (e < c) // 0 < t < 1
                    {
                        t = e / c;
                    }
                    else // t >= 1
                    {
                        t = one;
                        bmd = b - d;
                        if (bmd >= a)
                        {
                            s = one;
                        }
                        else if (bmd > zero)
                        {
                            s = bmd / a;
                        }
                    }
                }
                else // s > 0
                {
                    s = bte - ctd;
                    if (s >= det) // s >= 1
                    {
                        s = one;
                        bpe = b + e;
                        if (bpe <= zero) // t <= 0
                        {
                            t = zero;
                            nd = -d;
                            if (nd <= zero)
                            {
                                s = zero;
                            }
                            else if (nd < a)
                            {
                                s = nd / a;
                            }
                        }
                        else if (bpe < c) // 0 < t < 1
                        {
                            t = bpe / c;
                        }
                        else // t >= 1
                        {
                            t = one;
                            bmd = b - d;
                            if (bmd <= zero)
                            {
                                s = zero;
                            }
                            else if (bmd < a)
                            {
                                s = bmd / a;
                            }
                        }
                    }
                    else // 0 < s < 1
                    {
                        ate = a * e;
                        btd = b * d;
                        if (ate <= btd) // t <= 0
                        {
                            t = zero;
                            nd = -d;
                            if (nd <= zero)
                            {
                                s = zero;
                            }
                            else if (nd >= a)
                            {
                                s = one;
                            }
                            else
                            {
                                s = nd / a;
                            }
                        }
                        else // t > 0
                        {
                            t = ate - btd;
                            if (t >= det) // t >= 1
                            {
                                t = one;
                                bmd = b - d;
                                if (bmd <= zero)
                                {
                                    s = zero;
                                }
                                else if (bmd >= a)
                                {
                                    s = one;
                                }
                                else
                                {
                                    s = bmd / a;
                                }
                            }
                            else // 0 < t < 1
                            {
                                s /= det;
                                t /= det;
                            }
                        }
                    }
                }
            }
            else
            {
                // Parallel case.
                if (e <= zero) // t <= 0
                {
                    t = zero;
                    nd = -d;
                    if (nd <= zero)
                    {
                        s = zero;
                    }
                    else if (nd >= a)
                    {
                        s = one;
                    }
                    else
                    {
                        s = nd / a;
                    }
                }
                else if (e >= c) // t >= 1
                {
                    t = one;
                    bmd = b - d;
                    if (bmd <= zero)
                    {
                        s = zero;
                    }
                    else if (bmd >= a)
                    {
                        s = one;
                    }
                    else
                    {
                        s = bmd / a;
                    }
                }
                else // 0 < t < 1
                {
                    s = zero;
                    t = e / c;
                }
            }

            var result = new LineLineIsect
            {
                param1 = s,
                param2 = t,
                point1 = p0 + s * p1mp0,
                point2 = q0 + t * q1mq0
            };

            // Issue #1665 behavior in C++: distance within common plane.
            var diff = result.point1 - result.point2;
            var totalDist = Math.Sqrt(Vec3.Dot(diff, diff));

            if (totalDist < 1E-12)
            {
                result.distance = totalDist;
                return result;
            }

            if (totalDist > Eps.TOLERANCE_SCALAR_EQUALITY * 10)
            {
                result.distance = totalDist;
                return result;
            }

            var v1 = Vec3.Normalize(p1mp0);
            var v2 = Vec3.Normalize(q1mq0);
            if (Math.Abs(Vec3.Dot(v1, v2)) > 1 - Eps._TOLERANCE_PLANE_DEVIATION)
            {
                result.distance = totalDist;
                return result;
            }

            var normal = Vec3.Cross(p1mp0, q1mq0);
            var normalLength = normal.Length();
            normal = normal / normalLength;

            var perpDist = Math.Abs(Vec3.Dot(diff, normal));
            var subs = totalDist * totalDist - perpDist * perpDist;
            if (subs < 0)
            {
                subs = 0;
            }

            result.distance = Math.Sqrt(subs);
            return result;
        }

        public static PlanePlaneIsectResult PlanePlaneIsect(Vec3 norm1, double d1, Vec3 norm2, double d2)
        {
            var result = new PlanePlaneIsectResult();

            // https://stackoverflow.com/questions/6408670/line-of-intersection-between-two-planes
            result.dir = Vec3.Cross(norm1, norm2);
            var det = result.dir.Length();

            var det2 = new Mat3d(
                norm1.X, norm1.Y, norm1.Z,
                norm2.X, norm2.Y, norm2.Z,
                result.dir.X, result.dir.Y, result.dir.Z).Determinant;
            _ = det2;

            det *= det;
            result.pos = (Vec3.Cross(result.dir, norm2) * -d1 + Vec3.Cross(norm1, result.dir) * -d2) / det;
            result.dir = Vec3.Normalize(result.dir);

            return result;
        }

        // https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
        public static double DistancePointToLineSegment2D(Vec2 v, Vec2 w, Vec2 p)
        {
            // Return minimum distance between line segment vw and point p.
            var l2 = Vec2.Distance(v, w); // |w-v|
            if (l2 == 0.0)
            {
                return Vec2.Distance(p, v); // v == w case
            }

            var t = Math.Max(0.0, Math.Min(1.0, Vec2.Dot(p - v, w - v) / (l2 * l2)));
            var projection = v + t * (w - v);
            return Vec2.Distance(p, projection);
        }

        public static bool PointOnLineSegment2D(Vec2 v, Vec2 w, Vec2 p, double eps)
        {
            var dist = DistancePointToLineSegment2D(v, w, p);
            return dist <= eps;
        }

        public static bool onEdge2D(Vec2 p, Vec2 a, Vec2 b, double eps)
        {
            var dist = Math.Abs(sign2D(p, a, b));
            return dist <= eps;
        }

        public static bool IsVectorCCW(IReadOnlyList<Vec2> points)
        {
            var sum = 0.0;
            for (var i = 0; i < points.Count; i++)
            {
                var pt1 = points[i];
                var pt2 = points[(i + 1) % points.Count];
                sum += (pt2.X - pt1.X) * (pt2.Y + pt1.Y);
            }

            return sum < 0;
        }

        public static double areaOfTriangle(Vec2 a, Vec2 b, Vec2 c)
        {
            var ab = b - a;
            var ac = c - a;
            var norm = cross2d(ab, ac) / 2.0;
            return Math.Abs(norm);
        }

        // https://en.wikipedia.org/wiki/Barycentric_coordinate_system
        public static Vec3 ToBary(Vec3 a, Vec3 b, Vec3 c, Vec3 pt)
        {
            var e1 = b - a;
            var e2 = c - a;
            var rov0 = pt - a;
            var n = Vec3.Cross(e1, e2);
            var dir = n * -1.0;
            var q = Vec3.Cross(rov0, dir);
            var d = Vec3.Dot(dir, n);

            if (d == 0 && Eps.messages)
            {
                Console.WriteLine("bary conversion perp");
            }

            var det = 1.0 / d;
            var u = det * Vec3.Dot(e2, q * -1.0);
            var v = det * Vec3.Dot(e1, q);
            var w = 1 - u - v;

            return new Vec3(w, u, v);
        }

        public static Vec2 FromBary(Vec2 a, Vec2 b, Vec2 c, Vec3 pt)
        {
            return pt.X * a + pt.Y * b + pt.Z * c;
        }

        // assume 0,0 1,0 0,1 triangle
        public static Vec3 ToBary2(Vec2 pt)
        {
            var v = pt.X;
            var w = pt.Y;
            var u = 1 - v - w;
            return new Vec3(u, v, w);
        }

        public static Vec3 FromBary(Vec3 a, Vec3 b, Vec3 c, Vec3 pt)
        {
            return pt.X * a + pt.Y * b + pt.Z * c;
        }

        public static double RandomDouble(double lo, double hi)
        {
            return lo + Random.Shared.NextDouble() * (hi - lo);
        }
    }
}
