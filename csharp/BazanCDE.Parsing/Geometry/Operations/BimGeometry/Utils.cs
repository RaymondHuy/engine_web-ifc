using BazanCDE.Parsing.Utilities;

namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public enum Projection
    {
        XY,
        XZ,
        YZ
    }

    public readonly record struct Point3(double X, double Y, double Z)
    {
        public double this[int i] => i switch
        {
            0 => X,
            1 => Y,
            _ => Z
        };
    }

    public static partial class Utils
    {
        public const double CONST_PI = 3.141592653589793238462643383279502884;
        public const int VERTEX_FORMAT_SIZE_FLOATS = 6;

        private static readonly double[] Identity3 =
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };

        private static readonly double[] Identity4 =
        {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };

        public static double cross2d(Vec2 point1, Vec2 point2)
        {
            return point1.X * point2.Y - point1.Y * point2.X;
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

        public static double areaOfTriangle(Vec3 a, Vec3 b, Vec3 c)
        {
            var ab = b - a;
            var ac = c - a;
            var norm = Vec3.Cross(ab, ac);
            return norm.Length() / 2.0;
        }

        public static double areaOfTriangle2D(Vec2 a, Vec2 b, Vec2 c)
        {
            var ab = b - a;
            var ac = c - a;
            var norm = cross2d(ab, ac) / 2.0;
            return Math.Abs(norm);
        }

        public static bool computeSafeNormal(Vec3 v1, Vec3 v2, Vec3 v3, out Vec3 normal, double eps = 0)
        {
            var v12 = v2 - v1;
            var v13 = v3 - v1;
            var norm = Vec3.Cross(v12, v13);
            var len = norm.Length();

            if (len <= eps)
            {
                normal = new Vec3(0, 0, 0);
                return false;
            }

            normal = norm / len;
            return true;
        }

        public static double VectorToAngle(double x, double y)
        {
            var dd = Math.Sqrt(x * x + y * y);
            if (Math.Abs(dd) < Epsilons.EPS_MINISCULE)
            {
                return 0;
            }

            var xx = x / dd;
            var yy = y / dd;

            var angle = Math.Acos(xx);
            var cosv = Math.Cos(angle);
            var sinv = Math.Sin(angle);
            if (Math.Abs(xx - cosv) > 1e-5 || Math.Abs(yy - sinv) > 1e-5)
            {
                angle = Math.Asin(yy);
                sinv = Math.Sin(angle);
                cosv = Math.Cos(angle);
                if (Math.Abs(xx - cosv) > 1e-5 || Math.Abs(yy - sinv) > 1e-5)
                {
                    angle = angle + (CONST_PI - angle) * 2;
                    sinv = Math.Sin(angle);
                    cosv = Math.Cos(angle);
                    if (Math.Abs(xx - cosv) > 1e-5 || Math.Abs(yy - sinv) > 1e-5)
                    {
                        angle += CONST_PI;
                    }
                }
            }

            return (angle / (2 * CONST_PI)) * 360;
        }

        public static bool GetWindingOfTriangle(Vec3 a, Vec3 b, Vec3 c)
        {
            var v12 = b - a;
            var v13 = c - a;
            var norm = Vec3.Normalize(Vec3.Cross(v12, v13));
            return Vec3.Dot(norm, new Vec3(0, 0, 1)) > 0.0;
        }

        public static Geometry Revolution(double[] transform, double startDegrees, double endDegrees, List<Vec3> profile, double numRots)
        {
            var geometry = new Geometry();
            var m = Matrix4OrIdentity(transform);

            var cent = GetColumn4(m, 3);
            var vecX = Vec3.Normalize(GetColumn4(m, 0));
            var vecY = Vec3.Normalize(GetColumn4(m, 1));
            var vecZ = Vec3.Normalize(GetColumn4(m, 2));

            var rots = Math.Max(0, (int)numRots);
            var newPoints = new List<List<Vec3>>(rots);
            for (var r = 0; r < rots; r++)
            {
                newPoints.Add(new List<Vec3>());
            }

            var startRad = startDegrees / 180.0 * CONST_PI;
            var endRad = endDegrees / 180.0 * CONST_PI;
            var radSpan = endRad - startRad;
            var radStep = rots > 1 ? radSpan / (rots - 1) : 0.0;

            for (var i = 0; i < profile.Count; i++)
            {
                var xx = profile[i].X - cent.X;
                var yy = profile[i].Y - cent.Y;
                var zz = profile[i].Z - cent.Z;

                var dx = vecX.X * xx + vecX.Y * yy + vecX.Z * zz;
                var dy = vecY.X * xx + vecY.Y * yy + vecY.Z * zz;
                var dz = vecZ.X * xx + vecZ.Y * yy + vecZ.Z * zz;
                var dd = Math.Sqrt(dx * dx + dy * dy);

                for (var r = 0; r < rots; r++)
                {
                    var angle = startRad + r * radStep;
                    var dtempX = Math.Sin(angle) * dd;
                    var dtempY = Math.Cos(angle) * dd;
                    var newPx = dtempX * vecX.X + dtempY * vecY.X + dz * vecZ.X + cent.X;
                    var newPy = dtempX * vecX.Y + dtempY * vecY.Y + dz * vecZ.Y + cent.Y;
                    var newPz = dtempX * vecX.Z + dtempY * vecY.Z + dz * vecZ.Z + cent.Z;
                    newPoints[r].Add(new Vec3(newPx, newPy, newPz));
                }
            }

            for (var r = 0; r < rots - 1; r++)
            {
                var r1 = r + 1;
                if (r1 >= newPoints.Count)
                {
                    break;
                }

                var newPointsR = newPoints[r];
                var newPointsR1 = newPoints[r1];
                if (newPointsR.Count > 0)
                {
                    for (var s = 0; s < newPointsR.Count - 1; s++)
                    {
                        if (s + 1 >= newPointsR.Count || s + 1 >= newPointsR1.Count)
                        {
                            break;
                        }

                        geometry.AddFace(newPointsR[s], newPointsR[s + 1], newPointsR1[s], uint.MaxValue);
                        geometry.AddFace(newPointsR1[s], newPointsR[s + 1], newPointsR1[s + 1], uint.MaxValue);
                    }
                }
            }

            return geometry;
        }

        public static Geometry RevolveCylinder(double[] transform, double startDegrees, double endDegrees, double minZ, double maxZ, int numRots, double radius)
        {
            var geometry = new Geometry();
            var m = Matrix4OrIdentity(transform);

            var cent = GetColumn4(m, 3);
            var vecX = Vec3.Normalize(GetColumn4(m, 0));
            var vecY = Vec3.Normalize(GetColumn4(m, 1));
            var vecZ = Vec3.Normalize(GetColumn4(m, 2));

            var newPoints = new List<List<Vec3>>(numRots);
            for (var r = 0; r < numRots; r++)
            {
                newPoints.Add(new List<Vec3>());
            }

            var startRad = startDegrees / 180.0 * CONST_PI;
            var endRad = endDegrees / 180.0 * CONST_PI;
            var radSpan = endRad - startRad;
            var radStep = numRots > 1 ? radSpan / (numRots - 1) : 0.0;

            for (var r = 0; r < numRots; r++)
            {
                var angle = startRad + r * radStep;
                var dtempX = Math.Sin(angle) * radius;
                var dtempY = Math.Cos(angle) * radius;
                var newPx = dtempX * vecX.X + dtempY * vecY.X + minZ * vecZ.X + cent.X;
                var newPy = dtempX * vecX.Y + dtempY * vecY.Y + minZ * vecZ.Y + cent.Y;
                var newPz = dtempX * vecX.Z + dtempY * vecY.Z + minZ * vecZ.Z + cent.Z;
                newPoints[r].Add(new Vec3(newPx, newPy, newPz));
            }

            for (var r = 0; r < numRots; r++)
            {
                var angle = startRad + r * radStep;
                var dtempX = Math.Sin(angle) * radius;
                var dtempY = Math.Cos(angle) * radius;
                var newPx = dtempX * vecX.X + dtempY * vecY.X + maxZ * vecZ.X + cent.X;
                var newPy = dtempX * vecX.Y + dtempY * vecY.Y + maxZ * vecZ.Y + cent.Y;
                var newPz = dtempX * vecX.Z + dtempY * vecY.Z + maxZ * vecZ.Z + cent.Z;
                newPoints[r].Add(new Vec3(newPx, newPy, newPz));
            }

            for (var r = 0; r < numRots - 1; r++)
            {
                var r1 = r + 1;
                for (var s = 0; s < newPoints[r].Count - 1; s++)
                {
                    geometry.AddFace(newPoints[r][s], newPoints[r][s + 1], newPoints[r1][s], uint.MaxValue);
                    geometry.AddFace(newPoints[r1][s], newPoints[r][s + 1], newPoints[r1][s + 1], uint.MaxValue);
                }
            }

            return geometry;
        }

        public static bool MatrixFlipsTriangles(double[] mat)
        {
            if (mat.Length >= 16)
            {
                return Determinant4(Matrix4OrIdentity(mat)) < 0;
            }

            return Determinant3(Matrix3OrIdentity(mat)) < 0;
        }

        public static List<Vec3> Convert2DAlignmentsTo3D(IReadOnlyList<Vec3> horizontal, IReadOnlyList<Vec3> vertical)
        {
            var curve3D = new List<Vec3>();
            if (horizontal.Count == 0 || vertical.Count == 0)
            {
                return curve3D;
            }

            var length = 0.0;
            var lastX = horizontal[0].X;
            var lastY = horizontal[0].Y;

            for (var h = 0; h < horizontal.Count; h++)
            {
                var ptH = horizontal[h];
                var dx = ptH.X - lastX;
                var dy = ptH.Y - lastY;
                length += Math.Sqrt(dx * dx + dy * dy);
                lastX = ptH.X;
                lastY = ptH.Y;

                var altitude = 0.0;
                var lastAlt = vertical[0].Y;
                var lastVx = vertical[0].X;
                var found = false;

                for (var i = 1; i < vertical.Count; i++)
                {
                    var ptV = vertical[i];
                    if (ptV.X >= length)
                    {
                        var ratio = (length - lastVx) / (ptV.X - lastVx);
                        altitude = lastAlt * (1.0 - ratio) + ptV.Y * ratio;
                        found = true;
                        break;
                    }

                    lastAlt = ptV.Y;
                    lastVx = ptV.X;
                }

                if (!found)
                {
                    altitude = vertical[vertical.Count - 1].Y;
                }

                curve3D.Add(new Vec3(ptH.X, altitude, -ptH.Y));
            }

            return curve3D;
        }

        public static Curve GetEllipseCurve(
            float radiusX,
            float radiusY,
            int numSegments,
            double[]? placement = null,
            double startRad = 0,
            double endRad = CONST_PI * 2,
            bool swap = true,
            bool normalToCenterEnding = false)
        {
            var c = new Curve();
            var mat = Matrix3OrIdentity(placement);

            if (normalToCenterEnding)
            {
                var sweepAngle = endRad - startRad;
                var step = sweepAngle / (numSegments - 1);

                if (endRad > startRad)
                {
                    startRad -= step / 2;
                    endRad += step / 2;
                }

                if (endRad <= startRad)
                {
                    startRad += step / 2;
                    endRad -= step / 2;
                }

                for (var i = 0; i < numSegments; i++)
                {
                    var ratio = (double)i / (numSegments - 1);
                    var angle = startRad + ratio * (endRad - startRad);

                    Vec2 circleCoordinate;
                    if (swap)
                    {
                        circleCoordinate = new Vec2(radiusX * Math.Cos(angle), radiusY * Math.Sin(angle));
                    }
                    else
                    {
                        circleCoordinate = new Vec2(radiusX * Math.Sin(angle), radiusY * Math.Cos(angle));
                    }

                    var pos = TransformPoint3(mat, new Vec3(circleCoordinate.X, circleCoordinate.Y, 1));
                    c.points.Add(new Vec3(pos.X, pos.Y, 0));
                }

                if (c.points.Count > 1)
                {
                    c.points[0] = (c.points[0] + c.points[1]) * 0.5;
                    c.points[c.points.Count - 1] = (c.points[c.points.Count - 1] + c.points[c.points.Count - 2]) * 0.5;
                }

                if (endRad == CONST_PI * 2 && startRad == 0)
                {
                    c.points.Add(c.points[0]);
                    if (MatrixFlipsTriangles(mat))
                    {
                        c.Invert();
                    }
                }
            }
            else
            {
                for (var i = 0; i < numSegments; i++)
                {
                    var ratio = (double)i / (numSegments - 1);
                    var angle = startRad + ratio * (endRad - startRad);

                    Vec2 circleCoordinate;
                    if (swap)
                    {
                        circleCoordinate = new Vec2(radiusX * Math.Cos(angle), radiusY * Math.Sin(angle));
                    }
                    else
                    {
                        circleCoordinate = new Vec2(radiusX * Math.Sin(angle), radiusY * Math.Cos(angle));
                    }

                    var pos = TransformPoint3(mat, new Vec3(circleCoordinate.X, circleCoordinate.Y, 1));
                    c.points.Add(new Vec3(pos.X, pos.Y, 0));
                }

                if (endRad == CONST_PI * 2 && startRad == 0)
                {
                    c.points.Add(c.points[0]);
                    if (MatrixFlipsTriangles(mat))
                    {
                        c.Invert();
                    }
                }
            }

            return c;
        }

        public static List<Vec2> SolveParabola(
            ushort segments,
            Vec2 startPoint,
            double horizontalLength,
            double startHeight,
            double startGradient,
            double endGradient)
        {
            _ = startPoint;
            var points = new List<Vec2>();
            var r = horizontalLength / (endGradient - startGradient);

            for (var i = 0; i <= segments; i++)
            {
                var pr = (double)i / segments;
                var grad = ((horizontalLength * pr) / r) + startGradient;
                var alt = (horizontalLength * pr * (grad + startGradient) * 0.5) + startHeight;
                points.Add(new Vec2(horizontalLength * pr, alt));
            }

            return points;
        }

        public static List<Vec2> SolveClothoid(
            ushort segments,
            Vec2 startPoint,
            double ifcStartDirection,
            double startRadiusOfCurvature,
            double endRadiusOfCurvature,
            double segmentLength)
        {
            var points = new List<Vec2>();

            var inverse = false;
            if (Math.Abs(startRadiusOfCurvature) > Math.Abs(endRadiusOfCurvature))
            {
                inverse = true;
            }

            var a = Math.Sqrt(Math.Abs(endRadiusOfCurvature - startRadiusOfCurvature) * segmentLength);
            var api = a * Math.Sqrt(CONST_PI);
            var uMax = segmentLength / api;

            var s = a * uMax * Math.Sqrt(CONST_PI);
            _ = (a * a * a) / (a * s); // radFin (unused in C++)

            var vSin = 0.0;
            var vCos = 0.0;

            var directionX = new Vec2(Math.Cos(ifcStartDirection), Math.Sin(ifcStartDirection));
            var directionY = new Vec2(-Math.Sin(ifcStartDirection), Math.Cos(ifcStartDirection));

            if (endRadiusOfCurvature < 0 || startRadiusOfCurvature < 0)
            {
                directionY = new Vec2(-directionY.X, -directionY.Y);
            }

            if (inverse)
            {
                directionX = new Vec2(-directionX.X, -directionX.Y);
            }

            const double def = 2000;
            var dif = def / segments;
            var count = 0.0;
            var tram = uMax / (def - 1);
            var end = new Vec2(0, 0);
            var prev = new Vec2(0, 0);
            var endDir = new Vec2(0, 0);

            for (var c = 1.0; c < def + 1; c++)
            {
                prev = end;
                end = startPoint + api * (directionX * vCos + directionY * vSin);

                if (c == def || c == 1 || count >= dif)
                {
                    points.Add(end);
                    count = 0;
                }

                if (c == def)
                {
                    endDir = prev - end;
                }

                var val = c * tram;
                vSin += Math.Sin(CONST_PI * ((a * val * val) / (2 * Math.Abs(a)))) * tram;
                vCos += Math.Cos(CONST_PI * ((a * val * val) / (2 * Math.Abs(a)))) * tram;
                count++;
            }

            if (inverse)
            {
                directionX = new Vec2(-directionX.X, -directionX.Y);

                var newDirectionX = new Vec2(endDir.X, endDir.Y);
                var newDirectionY = new Vec2(-endDir.Y, endDir.X);

                if (endRadiusOfCurvature < 0 || startRadiusOfCurvature < 0)
                {
                    newDirectionY = new Vec2(-newDirectionY.X, -newDirectionY.Y);
                }

                newDirectionX = Vec2.Normalize(newDirectionX);
                newDirectionY = Vec2.Normalize(newDirectionY);

                for (var i = 0; i < points.Count; i++)
                {
                    var xx = points[i].X - end.X;
                    var yy = points[i].Y - end.Y;
                    var dx = xx * newDirectionX.X + yy * newDirectionX.Y;
                    var dy = xx * newDirectionY.X + yy * newDirectionY.Y;
                    var newDx = startPoint.X + directionX.X * dx + directionY.X * dy;
                    var newDy = startPoint.Y + directionX.Y * dx + directionY.Y * dy;
                    points[i] = new Vec2(newDx, newDy);
                }
            }

            return points;
        }

        public static Geometry Extrude(List<Vec3> points, Vec3 dir, double len)
        {
            var geom = new Geometry();

            for (var j = 0; j < points.Count - 1; j++)
            {
                var j2 = j + 1;

                var nptj1 = new Vec3(
                    points[j].X + dir.X * len,
                    points[j].Y + dir.Y * len,
                    points[j].Z + dir.Z * len);

                var nptj2 = new Vec3(
                    points[j2].X + dir.X * len,
                    points[j2].Y + dir.Y * len,
                    points[j2].Z + dir.Z * len);

                geom.AddFace(points[j], points[j2], nptj1, uint.MaxValue);
                geom.AddFace(points[j2], nptj2, nptj1, uint.MaxValue);
            }

            return geom;
        }

        public static Projection bestProjection(IReadOnlyList<Point3> poly)
        {
            static double Area2D(IReadOnlyList<Point3> p, int i1, int i2)
            {
                var area = 0.0;
                for (var i = 0; i < p.Count; ++i)
                {
                    var a = p[i];
                    var b = p[(i + 1) % p.Count];
                    area += (a[i1] * b[i2]) - (b[i1] * a[i2]);
                }

                return Math.Abs(area * 0.5);
            }

            var areaXY = Area2D(poly, 0, 1);
            var areaXZ = Area2D(poly, 0, 2);
            var areaYZ = Area2D(poly, 1, 2);

            if (areaXY >= areaXZ && areaXY >= areaYZ)
            {
                return Projection.XY;
            }

            if (areaXZ >= areaYZ)
            {
                return Projection.XZ;
            }

            return Projection.YZ;
        }

        public static List<List<Point3>> projectTo2D(IReadOnlyList<IReadOnlyList<Point3>> poly3D, Projection proj)
        {
            var poly2D = new List<List<Point3>>(poly3D.Count);

            for (var i = 0; i < poly3D.Count; ++i)
            {
                var ring = new List<Point3>(poly3D[i].Count);
                foreach (var pt in poly3D[i])
                {
                    switch (proj)
                    {
                        case Projection.XY:
                            ring.Add(new Point3(pt[0], pt[1], pt[2]));
                            break;
                        case Projection.XZ:
                            ring.Add(new Point3(pt[0], pt[2], pt[1]));
                            break;
                        case Projection.YZ:
                            ring.Add(new Point3(pt[1], pt[2], pt[0]));
                            break;
                    }
                }

                poly2D.Add(ring);
            }

            return poly2D;
        }

        public static Geometry Extrude(
            List<List<Vec3>> profile,
            Vec3 dir,
            double distance,
            Vec3? cuttingPlaneNormal = null,
            Vec3? cuttingPlanePos = null)
        {
            var geom = new Geometry();
            var holesIndicesHash = new List<bool>();
            var cuttingNormal = cuttingPlaneNormal ?? new Vec3(0, 0, 0);
            var cuttingPos = cuttingPlanePos ?? new Vec3(0, 0, 0);

            if (profile.Count == 0 || profile[0].Count == 0)
            {
                return geom;
            }

            var lastToFirstPoint = profile[0][0] - profile[0][profile[0].Count - 1];
            if (lastToFirstPoint.Length() > 1e-8)
            {
                profile[0].Add(profile[0][0]);
            }

            {
                var polygonCount = profile.Count;
                var polygon = new List<List<Point3>>(polygonCount);
                for (var i = 0; i < polygonCount; i++)
                {
                    polygon.Add(new List<Point3>());
                }

                var normal = dir;

                for (var i = 0; i < profile[0].Count; i++)
                {
                    var pt = profile[0][i];
                    var et = pt + dir * distance;
                    geom.AddPoint(et, normal);
                    polygon[0].Add(new Point3(pt.X, pt.Y, pt.Z));
                }

                for (var i = 0; i < profile[0].Count; i++)
                {
                    holesIndicesHash.Add(false);
                }

                for (var i = 1; i < profile.Count; i++)
                {
                    var hole = profile[i];
                    var pointCount = hole.Count;

                    for (var j = 0; j < pointCount; j++)
                    {
                        holesIndicesHash.Add(j == 0);

                        var pt = hole[j];
                        var et = pt + dir * distance;

                        profile[0].Add(pt);
                        geom.AddPoint(et, normal);
                        polygon[i].Add(new Point3(pt.X, pt.Y, pt.Z));
                    }
                }

                var proj = bestProjection(polygon[0]);
                var polygon2D = projectTo2D(polygon, proj);
                var indices = TriangulatePolygonRings(polygon2D);

                uint offset = 0;
                var flipWinding = false;

                if (indices.Count >= 3)
                {
                    var winding = GetWindingOfTriangle(
                        geom.GetPoint(offset + indices[0]),
                        geom.GetPoint(offset + indices[1]),
                        geom.GetPoint(offset + indices[2]));

                    flipWinding = !winding;

                    for (var i = 0; i < indices.Count; i += 3)
                    {
                        if (flipWinding)
                        {
                            geom.AddFace(offset + indices[i + 0], offset + indices[i + 2], offset + indices[i + 1], uint.MaxValue);
                        }
                        else
                        {
                            geom.AddFace(offset + indices[i + 0], offset + indices[i + 1], offset + indices[i + 2], uint.MaxValue);
                        }
                    }
                }

                offset += geom.numPoints;
                normal = dir * -1.0;

                for (var i = 0; i < profile[0].Count; i++)
                {
                    var pt = profile[0][i];
                    var et = pt;

                    if (cuttingNormal != new Vec3(0, 0, 0))
                    {
                        var transDir = dir;
                        var ldotn = Vec3.Dot(transDir, cuttingNormal);
                        if (ldotn != 0)
                        {
                            var dpos = cuttingPos - et;
                            var dist = Vec3.Dot(dpos, cuttingNormal) / ldotn;
                            et = et + dist * transDir;
                        }
                    }

                    geom.AddPoint(et, normal);
                }

                for (var i = 0; i < indices.Count; i += 3)
                {
                    if (flipWinding)
                    {
                        geom.AddFace(offset + indices[i + 0], offset + indices[i + 1], offset + indices[i + 2], uint.MaxValue);
                    }
                    else
                    {
                        geom.AddFace(offset + indices[i + 0], offset + indices[i + 2], offset + indices[i + 1], uint.MaxValue);
                    }
                }
            }

            var capSize = profile[0].Count;
            for (var i = 1; i < capSize; i++)
            {
                if (holesIndicesHash[i])
                {
                    continue;
                }

                var bl = (uint)(i - 1);
                var br = (uint)i;
                var tl = (uint)(capSize + i - 1);
                var tr = (uint)(capSize + i);

                geom.AddFace(geom.GetPoint(tl), geom.GetPoint(br), geom.GetPoint(bl), uint.MaxValue);
                geom.AddFace(geom.GetPoint(tl), geom.GetPoint(tr), geom.GetPoint(br), uint.MaxValue);
            }

            return geom;
        }

        public static Vec3 projectOntoPlane(Vec3 origin, Vec3 normal, Vec3 point, Vec3 dir)
        {
            var ldotn = Vec3.Dot(dir, normal);
            if (ldotn == 0)
            {
                return new Vec3(0, 0, 0);
            }

            var dpos = origin - point;
            var dist = Vec3.Dot(dpos, normal) / ldotn;
            return point + dist * dir;
        }

        public static Geometry SweepFunction(
            double scaling,
            bool closed,
            IReadOnlyList<Vec3> profilePoints,
            IReadOnlyList<Vec3> directrix,
            Vec3? initialDirectrixNormal = null,
            bool rotate90 = false,
            bool optimize = true)
        {
            var geom = new Geometry();
            var dpts = new List<Vec3>();

            for (var i = 0; i < directrix.Count; i++)
            {
                if (i < directrix.Count - 1)
                {
                    if ((directrix[i] - directrix[i + 1]).Length() > Epsilons.EPS_BIG2 * scaling || !optimize)
                    {
                        dpts.Add(directrix[i]);
                    }
                }
                else
                {
                    dpts.Add(directrix[i]);
                }
            }

            if (closed)
            {
                var dirStart = dpts[dpts.Count - 2] - dpts[dpts.Count - 1];
                var dirEnd = dpts[1] - dpts[0];
                var newDpts = new List<Vec3> { dpts[0] + dirStart };
                for (var i = 0; i < dpts.Count; i++)
                {
                    newDpts.Add(dpts[i]);
                }

                newDpts.Add(dpts[dpts.Count - 1] + dirEnd);
                dpts = newDpts;
            }

            if (dpts.Count <= 1)
            {
                return geom;
            }

            var curves = new List<Curve>();
            var initialNormal = initialDirectrixNormal ?? new Vec3(0, 0, 0);

            for (var i = 0; i < dpts.Count; i++)
            {
                var segmentForCurve = new Curve();

                Vec3 planeNormal;
                Vec3 directrixSegmentNormal;
                Vec3 planeOrigin;

                if (i == 0)
                {
                    planeNormal = Vec3.Normalize(dpts[1] - dpts[0]);
                    directrixSegmentNormal = planeNormal;
                    planeOrigin = dpts[0];
                }
                else if (i == dpts.Count - 1)
                {
                    planeNormal = Vec3.Normalize(dpts[i] - dpts[i - 1]);
                    directrixSegmentNormal = planeNormal;
                    planeOrigin = dpts[i];
                }
                else
                {
                    var n1 = Vec3.Normalize(dpts[i] - dpts[i - 1]);
                    var n2 = Vec3.Normalize(dpts[i + 1] - dpts[i]);
                    var p = Vec3.Normalize(Vec3.Cross(n1, n2));

                    var u1 = Vec3.Normalize(Vec3.Cross(n1, p));
                    var u2 = Vec3.Normalize(Vec3.Cross(n2, p));

                    if (Vec3.Dot(n1, n2) < -0.9)
                    {
                        n2 = n2 * -1.0;
                        u2 = u2 * -1.0;
                    }

                    var au = Vec3.Normalize(u1 + u2);
                    planeNormal = Vec3.Normalize(Vec3.Cross(au, p));
                    directrixSegmentNormal = n1;
                    planeOrigin = dpts[i];
                }

                if (curves.Count == 0)
                {
                    Vec3 left;
                    Vec3 right;
                    if (initialNormal == new Vec3(0, 0, 0))
                    {
                        left = Vec3.Cross(directrixSegmentNormal, new Vec3(directrixSegmentNormal.Y, directrixSegmentNormal.X, directrixSegmentNormal.Z));
                        if (left == new Vec3(0, 0, 0))
                        {
                            left = Vec3.Cross(directrixSegmentNormal, new Vec3(directrixSegmentNormal.X, directrixSegmentNormal.Z, directrixSegmentNormal.Y));
                        }

                        if (left == new Vec3(0, 0, 0))
                        {
                            left = Vec3.Cross(directrixSegmentNormal, new Vec3(directrixSegmentNormal.Z, directrixSegmentNormal.Y, directrixSegmentNormal.X));
                        }

                        right = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, left));
                        left = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, right));
                    }
                    else
                    {
                        left = Vec3.Cross(directrixSegmentNormal, initialNormal);
                        var side = Vec3.Normalize(initialNormal);
                        right = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, left));
                        left = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, right));
                        right = Hadamard(right, side);
                    }

                    for (var p = 0; p < profilePoints.Count; p++)
                    {
                        var pt2D = profilePoints[p];
                        var pt = (-pt2D.X * left) + (-pt2D.Y * right) + planeOrigin;
                        if (rotate90)
                        {
                            pt = (-pt2D.X * right) + (-pt2D.Y * left) + planeOrigin;
                        }

                        var proj = projectOntoPlane(planeOrigin, planeNormal, pt, directrixSegmentNormal);
                        segmentForCurve.Add(proj);
                    }
                }
                else
                {
                    var prevCurve = curves[curves.Count - 1];
                    for (var p = 0; p < prevCurve.points.Count; p++)
                    {
                        var proj = projectOntoPlane(planeOrigin, planeNormal, prevCurve.points[p], directrixSegmentNormal);
                        segmentForCurve.Add(proj);
                    }
                }

                if (!closed || (i != 0 && i != dpts.Count - 1))
                {
                    curves.Add(segmentForCurve);
                }
            }

            if (closed)
            {
                dpts.RemoveAt(dpts.Count - 1);
                dpts.RemoveAt(0);
            }

            for (var i = 1; i < dpts.Count; i++)
            {
                var c1 = curves[i - 1].points;
                var c2 = curves[i].points;
                var capSize = c1.Count;

                for (var j = 1; j < capSize; j++)
                {
                    var bl = c1[j - 1];
                    var br = c1[j - 0];
                    var tl = c2[j - 1];
                    var tr = c2[j - 0];

                    geom.AddFace(tl, br, bl, uint.MaxValue);
                    geom.AddFace(tl, tr, br, uint.MaxValue);
                }
            }

            return geom;
        }

        public static Geometry SweepCircular(
            double scaling,
            bool closed,
            IReadOnlyList<Vec3> profile,
            double radius,
            IReadOnlyList<Vec3> directrix,
            Vec3? initialDirectrixNormal = null,
            bool rotate90 = false)
        {
            var geom = new Geometry();
            var dpts = new List<Vec3>();

            for (var i = 0; i < directrix.Count; i++)
            {
                if (i < directrix.Count - 1)
                {
                    if ((directrix[i] - directrix[i + 1]).Length() > Epsilons.EPS_BIG2 * scaling)
                    {
                        dpts.Add(directrix[i]);
                    }
                }
                else
                {
                    dpts.Add(directrix[i]);
                }
            }

            if (closed)
            {
                var dirStart = dpts[dpts.Count - 2] - dpts[dpts.Count - 1];
                var dirEnd = dpts[1] - dpts[0];
                var newDpts = new List<Vec3> { dpts[0] + dirStart };
                for (var i = 0; i < dpts.Count; i++)
                {
                    newDpts.Add(dpts[i]);
                }

                newDpts.Add(dpts[dpts.Count - 1] + dirEnd);
                dpts = newDpts;
            }

            if (dpts.Count <= 1)
            {
                return geom;
            }

            var curves = new List<List<Vec3>>();
            var initialNormal = initialDirectrixNormal ?? new Vec3(0, 0, 0);

            for (var i = 0; i < dpts.Count; i++)
            {
                var segmentForCurve = new List<Vec3>();

                Vec3 directrix2;
                Vec3 planeNormal;
                Vec3 directrixSegmentNormal;
                Vec3 planeOrigin;

                if (i == 0)
                {
                    planeNormal = Vec3.Normalize(dpts[1] - dpts[0]);
                    directrixSegmentNormal = planeNormal;
                    planeOrigin = dpts[0];
                    directrix2 = planeNormal;
                }
                else if (i == dpts.Count - 1)
                {
                    planeNormal = Vec3.Normalize(dpts[i] - dpts[i - 1]);
                    directrixSegmentNormal = planeNormal;
                    planeOrigin = dpts[i];
                    directrix2 = planeNormal;
                }
                else
                {
                    var n1 = Vec3.Normalize(dpts[i] - dpts[i - 1]);
                    var n2 = Vec3.Normalize(dpts[i + 1] - dpts[i]);
                    var p = Vec3.Normalize(Vec3.Cross(n1, n2));
                    directrix2 = n1 * -1.0;

                    var u1 = Vec3.Normalize(Vec3.Cross(n1, p));
                    var u2 = Vec3.Normalize(Vec3.Cross(n2, p));

                    if (Vec3.Dot(n1, n2) < -0.9)
                    {
                        n2 = n2 * -1.0;
                        u2 = u2 * -1.0;
                    }

                    var au = Vec3.Normalize(u1 + u2);
                    planeNormal = Vec3.Normalize(Vec3.Cross(au, p));
                    directrixSegmentNormal = n1;
                    planeOrigin = dpts[i];
                }

                var dz = Vec3.Normalize(directrix2);
                Vec3 dx;
                var parallelZ = Math.Abs(Vec3.Dot(dz, new Vec3(0, 0, 1)));

                if (parallelZ > 1 - Epsilons.EPS_BIG2)
                {
                    dx = Vec3.Normalize(Vec3.Cross(dz, new Vec3(0, 1, 0)));
                }
                else
                {
                    dx = Vec3.Normalize(Vec3.Cross(dz, new Vec3(0, 0, 1)));
                }

                _ = Vec3.Normalize(Vec3.Cross(dz, dx));

                if (curves.Count == 0)
                {
                    Vec3 left;
                    Vec3 right;
                    if (initialNormal == new Vec3(0, 0, 0))
                    {
                        left = Vec3.Cross(directrixSegmentNormal, new Vec3(directrixSegmentNormal.Y, directrixSegmentNormal.X, directrixSegmentNormal.Z));
                        if (left == new Vec3(0, 0, 0))
                        {
                            left = Vec3.Cross(directrixSegmentNormal, new Vec3(directrixSegmentNormal.X, directrixSegmentNormal.Z, directrixSegmentNormal.Y));
                        }

                        if (left == new Vec3(0, 0, 0))
                        {
                            left = Vec3.Cross(directrixSegmentNormal, new Vec3(directrixSegmentNormal.Z, directrixSegmentNormal.Y, directrixSegmentNormal.X));
                        }

                        right = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, left));
                        left = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, right));
                    }
                    else
                    {
                        left = Vec3.Cross(directrixSegmentNormal, initialNormal);
                        var side = Vec3.Normalize(initialNormal);
                        right = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, left));
                        left = Vec3.Normalize(Vec3.Cross(directrixSegmentNormal, right));
                        right = Hadamard(right, side);
                    }

                    for (var p = 0; p < profile.Count; p++)
                    {
                        var pt2D = profile[p];
                        var pt = (-pt2D.X * left) + (-pt2D.Y * right) + planeOrigin;
                        if (rotate90)
                        {
                            pt = (-pt2D.X * right) + (-pt2D.Y * left) + planeOrigin;
                        }

                        var proj = projectOntoPlane(planeOrigin, planeNormal, pt, directrixSegmentNormal);
                        segmentForCurve.Add(proj);
                    }
                }
                else
                {
                    var prevCurve = curves[curves.Count - 1];
                    for (var p = 0; p < prevCurve.Count; p++)
                    {
                        var proj = projectOntoPlane(planeOrigin, planeNormal, prevCurve[p], directrixSegmentNormal);
                        segmentForCurve.Add(proj);
                    }
                }

                if (!closed || (i != 0 && i != dpts.Count - 1))
                {
                    curves.Add(segmentForCurve);
                }
            }

            if (closed)
            {
                dpts.RemoveAt(dpts.Count - 1);
                dpts.RemoveAt(0);
            }

            var curvePointIndices = new List<List<uint>>(curves.Count);
            for (var i = 0; i < curves.Count; ++i)
            {
                var pts = curves[i];
                var indices = new List<uint>(pts.Count);
                var center = dpts[i];

                for (var p = 0; p < pts.Count; p++)
                {
                    var n = pts[p] - center;
                    var len2 = Vec3.Dot(n, n);
                    n = len2 > 0.0 ? n / Math.Sqrt(len2) : new Vec3(0, 0, 1);
                    geom.AddPoint(pts[p], n);
                    indices.Add(geom.numPoints - 1);
                }

                curvePointIndices.Add(indices);
            }

            for (var i = 1; i < dpts.Count; i++)
            {
                var idx1 = curvePointIndices[i - 1];
                var idx2 = curvePointIndices[i];

                var capSize = idx1.Count;
                for (var j = 1; j < capSize; j++)
                {
                    var bl = idx1[j - 1];
                    var br = idx1[j - 0];
                    var tl = idx2[j - 1];
                    var tr = idx2[j - 0];

                    geom.AddFace(tl, br, bl, uint.MaxValue);
                    geom.AddFace(tl, tr, br, uint.MaxValue);
                }
            }

            return geom;
        }

        public static Geometry SectionedSurface(List<List<Vec3>> profiles, bool buildCaps, double eps = 0.0)
        {
            var geom = new Geometry();

            if (profiles.Count < 2)
            {
                return geom;
            }

            for (var i = 0; i < profiles.Count - 1; i++)
            {
                var profile1 = profiles[i];
                var profile2 = profiles[i + 1];

                if (profile1.Count == 0 || profile2.Count == 0)
                {
                    continue;
                }

                if (profile1.Count != profile2.Count)
                {
                    continue;
                }

                var indices = new List<uint>();

                for (var j = 0; j < profile1.Count; j++)
                {
                    var p1 = profile1[j];
                    var p2 = profile2[j];

                    var normal = new Vec3(0, 0, 1);
                    var edge = p2 - p1;
                    if (edge.Length() > eps)
                    {
                        var crossVec = Vec3.Cross(edge, new Vec3(0, 0, 1));
                        if (crossVec.Length() > eps)
                        {
                            normal = Vec3.Normalize(Vec3.Cross(edge, crossVec));
                        }
                    }

                    geom.AddPoint(p1, normal);
                    geom.AddPoint(p2, normal);

                    indices.Add(geom.numPoints - 2);
                    indices.Add(geom.numPoints - 1);
                }

                for (var j = 0; j < indices.Count - 2; j += 2)
                {
                    if (j + 3 < indices.Count)
                    {
                        geom.AddFace(indices[j], indices[j + 1], indices[j + 2], uint.MaxValue);
                        geom.AddFace(indices[j + 1], indices[j + 3], indices[j + 2], uint.MaxValue);
                    }
                }
            }

            if (buildCaps && profiles.Count >= 2)
            {
                var capCandidates = new[] { 0, profiles.Count - 1 };
                for (var capIdxIndex = 0; capIdxIndex < capCandidates.Length; capIdxIndex++)
                {
                    var capIdx = capCandidates[capIdxIndex];
                    var profile = profiles[capIdx];
                    if (profile.Count < 3)
                    {
                        continue;
                    }

                    var poly3D = new List<Point3>(profile.Count);
                    for (var i = 0; i < profile.Count; i++)
                    {
                        var p = profile[i];
                        poly3D.Add(new Point3(p.X, p.Y, p.Z));
                    }

                    var proj = bestProjection(poly3D);
                    var poly2D = projectTo2D(new List<IReadOnlyList<Point3>> { poly3D }, proj);
                    var capIndices = TriangulatePolygonRings(poly2D);

                    var avgNormal = new Vec3(0, 0, 0);
                    for (var i = 0; i < profile.Count; ++i)
                    {
                        var p1 = profile[i];
                        var p2 = profile[(i + 1) % profile.Count];
                        var edge = p2 - p1;
                        var crossVec = Vec3.Cross(edge, new Vec3(0, 0, 1));
                        if (crossVec.Length() > eps)
                        {
                            avgNormal += Vec3.Normalize(crossVec);
                        }
                    }

                    if (avgNormal.Length() < eps)
                    {
                        avgNormal = new Vec3(0, 0, 1);
                    }
                    else
                    {
                        avgNormal = Vec3.Normalize(avgNormal);
                    }

                    var baseIndex = geom.numPoints;
                    for (var i = 0; i < profile.Count; i++)
                    {
                        var normal = capIdx == 0 ? avgNormal * -1.0 : avgNormal;
                        geom.AddPoint(profile[i], normal);
                    }

                    for (var i = 0; i < capIndices.Count; i += 3)
                    {
                        var i0 = baseIndex + capIndices[i];
                        var i1 = baseIndex + capIndices[i + 1];
                        var i2 = baseIndex + capIndices[i + 2];

                        if (capIdx == 0)
                        {
                            geom.AddFace(i0, i2, i1, uint.MaxValue);
                        }
                        else
                        {
                            geom.AddFace(i0, i1, i2, uint.MaxValue);
                        }
                    }
                }
            }

            return geom;
        }

        public static Curve GetRectangleCurve(double xdim, double ydim, double[]? placement = null, int numSegments = 12, double radius = 0)
        {
            var mat = Matrix4OrIdentity(placement);

            if (radius == 0)
            {
                var halfX = xdim / 2;
                var halfY = ydim / 2;

                var bl = new Vec2(-halfX, -halfY);
                var br = new Vec2(halfX, -halfY);
                var tl = new Vec2(-halfX, halfY);
                var tr = new Vec2(halfX, halfY);

                var c = new Curve();
                c.Add(TransformPoint4(mat, new Vec3(bl.X, bl.Y, 0)));
                c.Add(TransformPoint4(mat, new Vec3(br.X, br.Y, 0)));
                c.Add(TransformPoint4(mat, new Vec3(tr.X, tr.Y, 0)));
                c.Add(TransformPoint4(mat, new Vec3(tl.X, tl.Y, 0)));
                c.Add(TransformPoint4(mat, new Vec3(bl.X, bl.Y, 0)));

                if (MatrixFlipsTriangles(mat))
                {
                    c.Invert();
                }

                return c;
            }
            else
            {
                var halfX = xdim / 2;
                var halfY = ydim / 2;

                var bl = new Vec2(-halfX + radius, -halfY + radius);
                var br = new Vec2(halfX - radius, -halfY + radius);
                var tl = new Vec2(-halfX + radius, halfY - radius);
                var tr = new Vec2(halfX - radius, halfY - radius);

                var placement1 = Matrix3OrIdentity(null);
                placement1[6] = bl.X;
                placement1[7] = bl.Y;

                var placement2 = Matrix3OrIdentity(null);
                placement2[6] = br.X;
                placement2[7] = br.Y;

                var placement3 = Matrix3OrIdentity(null);
                placement3[6] = tl.X;
                placement3[7] = tl.Y;

                var placement4 = Matrix3OrIdentity(null);
                placement4[6] = tr.X;
                placement4[7] = tr.Y;

                var round1 = GetEllipseCurve((float)radius, (float)radius, numSegments, placement1, CONST_PI, 3 * CONST_PI / 2).points;
                var round2 = GetEllipseCurve((float)radius, (float)radius, numSegments, placement2, 3 * CONST_PI / 2, 2 * CONST_PI).points;
                var round3 = GetEllipseCurve((float)radius, (float)radius, numSegments, placement3, CONST_PI / 2, CONST_PI).points;
                var round4 = GetEllipseCurve((float)radius, (float)radius, numSegments, placement4, 0, CONST_PI / 2).points;

                var c = new Curve();
                for (var i = 0; i < round1.Count; i++)
                {
                    c.Add(TransformPoint4(mat, round1[i]));
                }

                for (var i = 0; i < round2.Count; i++)
                {
                    c.Add(TransformPoint4(mat, round2[i]));
                }

                for (var i = 0; i < round4.Count; i++)
                {
                    c.Add(TransformPoint4(mat, round4[i]));
                }

                for (var i = 0; i < round3.Count; i++)
                {
                    c.Add(TransformPoint4(mat, round3[i]));
                }

                if (round1.Count > 0)
                {
                    c.Add(TransformPoint4(mat, round1[0]));
                }

                if (MatrixFlipsTriangles(mat))
                {
                    c.Invert();
                }

                return c;
            }
        }

        public static Curve GetIShapedCurve(
            double width,
            double depth,
            double webThickness,
            double flangeThickness,
            bool hasFillet,
            double filletRadius,
            double[]? placement = null)
        {
            var c = new Curve();
            var mat = Matrix4OrIdentity(placement);

            var hw = width / 2;
            var hd = depth / 2;
            var hweb = webThickness / 2;

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, +hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(+hw, +hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(+hw, +hd - flangeThickness, 0)));

            if (hasFillet)
            {
                c.points.Add(TransformPoint4(mat, new Vec3(+hweb + filletRadius, +hd - flangeThickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(+hweb, +hd - flangeThickness - filletRadius, 0)));

                c.points.Add(TransformPoint4(mat, new Vec3(+hweb, -hd + flangeThickness + filletRadius, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(+hweb + filletRadius, -hd + flangeThickness, 0)));
            }
            else
            {
                c.points.Add(TransformPoint4(mat, new Vec3(+hweb, +hd - flangeThickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(+hweb, -hd + flangeThickness, 0)));
            }

            c.points.Add(TransformPoint4(mat, new Vec3(+hw, -hd + flangeThickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(+hw, -hd, 0)));

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, -hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw, -hd + flangeThickness, 0)));

            if (hasFillet)
            {
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb - filletRadius, -hd + flangeThickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, -hd + flangeThickness + filletRadius, 0)));

                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, +hd - flangeThickness - filletRadius, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb - filletRadius, +hd - flangeThickness, 0)));
            }
            else
            {
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, -hd + flangeThickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, +hd - flangeThickness, 0)));
            }

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, +hd - flangeThickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw, +hd, 0)));

            if (MatrixFlipsTriangles(mat))
            {
                c.Invert();
            }

            return c;
        }

        public static Curve GetUShapedCurve(
            double depth,
            double flangeWidth,
            double webThickness,
            double flangeThickness,
            double filletRadius,
            double edgeRadius,
            double flangeSlope,
            double[]? placement = null)
        {
            _ = filletRadius;
            _ = edgeRadius;
            var c = new Curve();
            var mat = Matrix4OrIdentity(placement);

            var hd = depth / 2;
            var hw = flangeWidth / 2;
            var slopeOffsetRight = flangeSlope * hw;
            var slopeOffsetLeft = flangeSlope * (hw - webThickness);

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, +hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(+hw, +hd, 0)));

            c.points.Add(TransformPoint4(mat, new Vec3(+hw, +hd - flangeThickness + slopeOffsetRight, 0)));

            c.points.Add(TransformPoint4(mat, new Vec3(-hw + webThickness, +hd - flangeThickness, -slopeOffsetLeft)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw + webThickness, -hd + flangeThickness, +slopeOffsetLeft)));

            c.points.Add(TransformPoint4(mat, new Vec3(+hw, -hd + flangeThickness - slopeOffsetRight, 0)));

            c.points.Add(TransformPoint4(mat, new Vec3(+hw, -hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw, -hd, 0)));

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, +hd, 0)));

            if (MatrixFlipsTriangles(mat))
            {
                c.Invert();
            }

            return c;
        }

        public static Curve GetLShapedCurve(
            double width,
            double depth,
            double thickness,
            bool hasFillet,
            double filletRadius,
            double edgeRadius,
            double legSlope,
            int numSegments = 12,
            double[]? placement = null)
        {
            _ = edgeRadius;
            _ = legSlope;
            var c = new Curve();
            var mat = Matrix4OrIdentity(placement);

            var hw = width / 2;
            var hd = depth / 2;

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, +hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw, -hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(+hw, -hd, 0)));

            if (hasFillet)
            {
                var secondFilletRadius = filletRadius;
                if (thickness < filletRadius)
                {
                    secondFilletRadius = thickness;
                }

                var cen1 = new Vec3(+hw - filletRadius, -hd + thickness - secondFilletRadius, 0);
                var placement1 = Matrix3OrIdentity(null);
                placement1[6] = cen1.X;
                placement1[7] = cen1.Y;
                var round1 = GetEllipseCurve((float)filletRadius, (float)secondFilletRadius, numSegments, placement1, 0, CONST_PI / 2).points;
                for (var i = 0; i < round1.Count; i++)
                {
                    c.Add(TransformPoint4(mat, round1[i]));
                }

                var cen2 = new Vec3(-hw + thickness + filletRadius, -hd + thickness + filletRadius, 0);
                var placement2 = Matrix3OrIdentity(null);
                placement2[6] = cen2.X;
                placement2[7] = cen2.Y;
                var round2 = GetEllipseCurve((float)filletRadius, (float)filletRadius, numSegments, placement2, 3 * CONST_PI / 2, CONST_PI).points;
                for (var i = 0; i < round2.Count; i++)
                {
                    c.Add(TransformPoint4(mat, round2[i]));
                }

                var cen3 = new Vec3(-hw + thickness - secondFilletRadius, hd - filletRadius, 0);
                var placement3 = Matrix3OrIdentity(null);
                placement3[6] = cen3.X;
                placement3[7] = cen3.Y;
                var round3 = GetEllipseCurve((float)secondFilletRadius, (float)filletRadius, numSegments, placement3, 0, CONST_PI / 2).points;
                for (var i = 0; i < round3.Count; i++)
                {
                    c.Add(TransformPoint4(mat, round3[i]));
                }
            }
            else
            {
                c.points.Add(TransformPoint4(mat, new Vec3(+hw, -hd + thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hw + thickness, -hd + thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hw + thickness, +hd, 0)));
            }

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, +hd, 0)));

            if (MatrixFlipsTriangles(mat))
            {
                c.Invert();
            }

            return c;
        }

        public static Curve GetTShapedCurve(
            double width,
            double depth,
            double thickness,
            bool hasFillet,
            double filletRadius,
            double edgeRadius,
            double legSlope,
            double[]? placement = null)
        {
            _ = filletRadius;
            _ = edgeRadius;
            _ = legSlope;

            var c = new Curve();
            var mat = Matrix4OrIdentity(placement);

            var hw = width / 2;
            var hd = depth / 2;
            var hweb = thickness / 2;

            c.points.Add(TransformPoint4(mat, new Vec3(hw, hd, 0)));

            if (hasFillet)
            {
                c.points.Add(TransformPoint4(mat, new Vec3(hw, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(hweb, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(hweb, -hd, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, -hd, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd, 0)));
            }
            else
            {
                c.points.Add(TransformPoint4(mat, new Vec3(hw, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(hweb, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(hweb, -hd, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, -hd, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hweb, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd - thickness, 0)));
                c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd, 0)));
            }

            c.points.Add(TransformPoint4(mat, new Vec3(hw, hd, 0)));

            if (MatrixFlipsTriangles(mat))
            {
                c.Invert();
            }

            return c;
        }

        public static Curve GetCShapedCurve(
            double width,
            double depth,
            double girth,
            double thickness,
            bool hasFillet,
            double filletRadius,
            double[]? placement = null)
        {
            _ = hasFillet;
            _ = filletRadius;

            var c = new Curve();
            var mat = Matrix4OrIdentity(placement);

            var hw = width / 2;
            var hd = depth / 2;

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw, hd, 0)));

            c.points.Add(TransformPoint4(mat, new Vec3(hw, hd - girth, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw - thickness, hd - girth, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw - thickness, hd - thickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw + thickness, hd - thickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw + thickness, -hd + thickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw - thickness, -hd + thickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw - thickness, -hd + girth, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw, -hd + girth, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw, -hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw, -hd, 0)));

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd, 0)));

            if (MatrixFlipsTriangles(mat))
            {
                c.Invert();
            }

            return c;
        }

        public static Curve GetZShapedCurve(
            double depth,
            double flangeWidth,
            double webThickness,
            double flangeThickness,
            double filletRadius,
            double edgeRadius,
            double[]? placement = null)
        {
            _ = filletRadius;
            _ = edgeRadius;

            var c = new Curve();
            var mat = Matrix4OrIdentity(placement);

            var hw = flangeWidth / 2;
            var hd = depth / 2;
            var hweb = webThickness / 2;

            c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hweb, hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hweb, -hd + flangeThickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw, -hd + flangeThickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(hw, -hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hweb, -hd, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hweb, hd - flangeThickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd - flangeThickness, 0)));
            c.points.Add(TransformPoint4(mat, new Vec3(-hw, hd, 0)));

            if (MatrixFlipsTriangles(mat))
            {
                c.Invert();
            }

            return c;
        }

        public static Curve GetTrapeziumCurve(
            double bottomXDim,
            double topXDim,
            double yDim,
            double topXOffset,
            double[]? placement = null)
        {
            var mat = Matrix4OrIdentity(placement);

            var halfX = bottomXDim / 2;
            var halfY = yDim / 2;

            var bl3 = TransformPoint4(mat, new Vec3(-halfX, -halfY, 0));
            var br3 = TransformPoint4(mat, new Vec3(halfX, -halfY, 0));
            var tl3 = TransformPoint4(mat, new Vec3(-halfX + topXOffset, halfY, 0));
            var tr3 = TransformPoint4(mat, new Vec3(-halfX + topXOffset + topXDim, halfY, 0));

            var bl = new Vec2(bl3.X, bl3.Y);
            var br = new Vec2(br3.X, br3.Y);
            var tl = new Vec2(tl3.X, tl3.Y);
            var tr = new Vec2(tr3.X, tr3.Y);

            var c = new Curve();
            c.Add(bl);
            c.Add(br);
            c.Add(tr);
            c.Add(tl);
            c.Add(bl);

            if (MatrixFlipsTriangles(mat))
            {
                c.Invert();
            }

            return c;
        }

        public static void SetEpsilons(double toleranceScalarEquality, double planeRefitIterations, double booleanUnionThreshold)
        {
            Epsilons._TOLERANCE_SCALAR_EQUALITY = toleranceScalarEquality;

            if (planeRefitIterations < 1)
            {
                planeRefitIterations = 1;
            }

            Epsilons._PLANE_REFIT_ITERATIONS = planeRefitIterations;

            if (booleanUnionThreshold < 1)
            {
                booleanUnionThreshold = 1;
            }

            Epsilons._BOOLEAN_UNION_THRESHOLD = booleanUnionThreshold;
        }

        private static double[] Matrix3OrIdentity(double[]? matrix)
        {
            if (matrix is null)
            {
                var clone = new double[9];
                Array.Copy(Identity3, clone, 9);
                return clone;
            }

            if (matrix.Length < 9)
            {
                throw new ArgumentException("3x3 matrix must contain at least 9 values.");
            }

            return matrix;
        }

        private static double[] Matrix4OrIdentity(double[]? matrix)
        {
            if (matrix is null)
            {
                var clone = new double[16];
                Array.Copy(Identity4, clone, 16);
                return clone;
            }

            if (matrix.Length < 16)
            {
                throw new ArgumentException("4x4 matrix must contain at least 16 values.");
            }

            return matrix;
        }

        private static Vec3 TransformPoint3(double[] m, Vec3 v)
        {
            var x = m[0] * v.X + m[3] * v.Y + m[6] * v.Z;
            var y = m[1] * v.X + m[4] * v.Y + m[7] * v.Z;
            var z = m[2] * v.X + m[5] * v.Y + m[8] * v.Z;
            return new Vec3(x, y, z);
        }

        private static Vec3 TransformPoint4(double[] m, Vec3 v)
        {
            var x = m[0] * v.X + m[4] * v.Y + m[8] * v.Z + m[12];
            var y = m[1] * v.X + m[5] * v.Y + m[9] * v.Z + m[13];
            var z = m[2] * v.X + m[6] * v.Y + m[10] * v.Z + m[14];
            var w = m[3] * v.X + m[7] * v.Y + m[11] * v.Z + m[15];

            if (Math.Abs(w) > Epsilons.EPS_NONZERO && Math.Abs(w - 1.0) > Epsilons.EPS_NONZERO)
            {
                return new Vec3(x / w, y / w, z / w);
            }

            return new Vec3(x, y, z);
        }

        private static Vec3 GetColumn4(double[] m, int column)
        {
            var o = column * 4;
            return new Vec3(m[o + 0], m[o + 1], m[o + 2]);
        }

        private static Vec3 Hadamard(Vec3 a, Vec3 b)
        {
            return new Vec3(a.X * b.X, a.Y * b.Y, a.Z * b.Z);
        }

        private static double Determinant3(double[] m)
        {
            var a00 = m[0];
            var a01 = m[3];
            var a02 = m[6];
            var a10 = m[1];
            var a11 = m[4];
            var a12 = m[7];
            var a20 = m[2];
            var a21 = m[5];
            var a22 = m[8];

            return a00 * (a11 * a22 - a12 * a21)
                 - a01 * (a10 * a22 - a12 * a20)
                 + a02 * (a10 * a21 - a11 * a20);
        }

        private static double Determinant4(double[] m)
        {
            var a00 = m[0];
            var a01 = m[4];
            var a02 = m[8];
            var a03 = m[12];
            var a10 = m[1];
            var a11 = m[5];
            var a12 = m[9];
            var a13 = m[13];
            var a20 = m[2];
            var a21 = m[6];
            var a22 = m[10];
            var a23 = m[14];
            var a30 = m[3];
            var a31 = m[7];
            var a32 = m[11];
            var a33 = m[15];

            var b00 = a00 * a11 - a01 * a10;
            var b01 = a00 * a12 - a02 * a10;
            var b02 = a00 * a13 - a03 * a10;
            var b03 = a01 * a12 - a02 * a11;
            var b04 = a01 * a13 - a03 * a11;
            var b05 = a02 * a13 - a03 * a12;
            var b06 = a20 * a31 - a21 * a30;
            var b07 = a20 * a32 - a22 * a30;
            var b08 = a20 * a33 - a23 * a30;
            var b09 = a21 * a32 - a22 * a31;
            var b10 = a21 * a33 - a23 * a31;
            var b11 = a22 * a33 - a23 * a32;

            return b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
        }

        private static List<uint> TriangulatePolygonRings(IReadOnlyList<IReadOnlyList<Point3>> rings)
        {
            var data = new List<double>();
            var holes = new List<int>();

            var vertexCount = 0;
            for (var r = 0; r < rings.Count; r++)
            {
                if (r > 0)
                {
                    holes.Add(vertexCount);
                }

                var ring = rings[r];
                for (var i = 0; i < ring.Count; i++)
                {
                    data.Add(ring[i].X);
                    data.Add(ring[i].Y);
                }

                vertexCount += ring.Count;
            }

            var tri = EarCut.Earcut(data, holes.Count == 0 ? null : holes, 2);
            var indices = new List<uint>(tri.Count);
            for (var i = 0; i < tri.Count; i++)
            {
                indices.Add((uint)tri[i]);
            }

            return indices;
        }
    }
}
