using System;
using System.Collections.Generic;

namespace BazanCDE.Parsing.Geometry.Operations.BooleanUtils
{
    public struct PlaneBasis
    {
        public Vec3 origin;
        public Vec3 up;
        public Vec3 left;
        public Vec3 right;

        public Vec2 project(Vec3 pt)
        {
            var relative = pt - origin;
            return new Vec2(Vec3.Dot(relative, left), Vec3.Dot(relative, right));
        }
    }

    public struct ReferencePlane
    {
        public int planeID;
        public int pointID;
        public int lineID;
        public Vec2 location;
    }

    public struct ReferenceLine
    {
        public int lineID;
        public int pointID;
        public double location;
    }

    /*
        The vector direction is not necessarily a unit vector! It points from origin to end.
        end = origin + direction
    */
    public sealed class Line
    {
        private static int _idCounter;

        public int id;
        public int globalID;
        public Vec3 origin;
        public Vec3 direction;

        public readonly List<(double Dist, int PointId)> points = new();
        public readonly List<ReferencePlane> planes = new();

        public Line()
        {
            globalID = ++_idCounter;
        }

        public bool IsPointOnLine(Vec3 a)
        {
            var d = Vec3.Normalize(direction);
            var v = a - origin;
            var t = Vec3.Dot(v, d);
            var p = origin + t * d;
            return (p - a).Length() < Eps.tolerancePointOnLine;
        }

        public double GetPosOnLine(Vec3 pos)
        {
            var unitDirection = Vec3.Normalize(direction);
            return Vec3.Dot(pos - origin, unitDirection);
        }

        public Vec3 GetPosOnLine(double dist)
        {
            var unitDirection = Vec3.Normalize(direction);
            return origin + dist * unitDirection;
        }

        public bool IsCollinear(Line other)
        {
            var unitDirection = Vec3.Normalize(direction);
            var unitOtherDirection = Vec3.Normalize(other.direction);
            return Utils.equals(unitOtherDirection, unitDirection, Eps.toleranceCollinear)
                   || Utils.equals(unitOtherDirection, unitDirection * -1.0, Eps.toleranceCollinear);
        }

        public bool IsEqualTo(Vec3 pos, Vec3 dir)
        {
            var unitDir = Vec3.Normalize(dir);
            var unitDirection = Vec3.Normalize(direction);
            if (!(Utils.equals(unitDir, unitDirection, Eps.EPS_SMALL)
                  || Utils.equals(unitDir, unitDirection * -1.0, Eps.EPS_SMALL)))
            {
                return false;
            }

            if (!IsPointOnLine(pos))
            {
                return false;
            }

            return true;
        }

        public void AddPointToLine(double dist, int pointId)
        {
            for (var i = 0; i < points.Count; i++)
            {
                if (points[i].PointId == pointId)
                {
                    return;
                }
            }

            points.Add((dist, pointId));
            points.Sort((left, right) => left.Dist.CompareTo(right.Dist));
        }

        public IEnumerable<(int First, int Second)> GetSegments()
        {
            for (var i = 1; i < points.Count; i++)
            {
                yield return (points[i - 1].PointId, points[i].PointId);
            }
        }
    }

    public sealed class Point
    {
        private static int _idCounter;

        public int id;
        public int globalID;
        public Vec3 location3D;

        public readonly List<ReferenceLine> lines = new();
        public readonly List<ReferencePlane> planes = new();

        public Point()
        {
            globalID = ++_idCounter;
        }

        public bool Matches(Vec3 pt)
        {
            return Utils.equals(location3D, pt, Eps.toleranceVectorEquality);
        }
    }

    public sealed class Plane
    {
        private static int _idCounter;

        public int refPlane = -1;
        public int id;
        public int globalID;
        public double distance;
        public Vec3 normal;

        public readonly List<Line> lines = new();
        public readonly AABB aabb = new();

        public Plane()
        {
            globalID = ++_idCounter;
        }

        public void AddPoint(Vec3 pt)
        {
            aabb.Merge(pt);
        }

        public double round(double input)
        {
            input = Math.Abs(input) < Eps.EPS_BIG ? 0.0 : input;
            input = Math.Abs(input) < (1.0 - Eps.EPS_BIG) ? input : input > 0.0 ? 1.0 : -1.0;
            return input;
        }

        public Vec3 round(Vec3 input)
        {
            return new Vec3(round(input.X), round(input.Y), round(input.Z));
        }

        public Vec3 GetDirection(Vec3 a, Vec3 b)
        {
            return Vec3.Normalize(b - a);
        }

        public (int LineId, bool Added) AddLine(Point a, Point b)
        {
            var pos = a.location3D;
            var dir = GetDirection(pos, b.location3D);

            var lineId = AddLine(pos, dir);

            if (!lines[lineId.LineId].IsPointOnLine(a.location3D) && Eps.messages)
            {
                Console.WriteLine("bad point in AddLine");
            }

            if (!lines[lineId.LineId].IsPointOnLine(b.location3D) && Eps.messages)
            {
                Console.WriteLine("bad point in AddLine");
            }

            if (!aabb.Contains(a.location3D) && Eps.messages)
            {
                Console.WriteLine("bad points in AddLine");
            }

            if (!aabb.Contains(b.location3D) && Eps.messages)
            {
                Console.WriteLine("bad points in AddLine");
            }

            lines[lineId.LineId].AddPointToLine(lines[lineId.LineId].GetPosOnLine(a.location3D), a.id);
            lines[lineId.LineId].AddPointToLine(lines[lineId.LineId].GetPosOnLine(b.location3D), b.id);

            return lineId;
        }

        public (int LineId, bool Added) AddLine(Vec3 pos, Vec3 dir)
        {
            for (var i = 0; i < lines.Count; i++)
            {
                var line = lines[i];
                if (!line.IsEqualTo(pos, dir))
                {
                    continue;
                }

                var hasPlaneRef = false;
                for (var j = 0; j < line.planes.Count; j++)
                {
                    if (line.planes[j].planeID == id)
                    {
                        hasPlaneRef = true;
                        break;
                    }
                }

                if (!hasPlaneRef)
                {
                    line.planes.Add(new ReferencePlane { planeID = id, lineID = line.id });
                }

                return (line.id, false);
            }

            var l = new Line
            {
                id = lines.Count,
                origin = pos,
                direction = Vec3.Normalize(dir)
            };
            l.planes.Add(new ReferencePlane { planeID = id, lineID = l.id });
            lines.Add(l);

            return (l.id, true);
        }

        public void RemoveLastLine()
        {
            if (lines.Count > 0)
            {
                lines.RemoveAt(lines.Count - 1);
            }
        }

        public bool IsEqualTo(Vec3 n, double d)
        {
            return Utils.equals(normal, n, Eps.toleranceVectorEquality)
                   && Utils.equals(distance, d, Eps.TOLERANCE_SCALAR_EQUALITY);
        }

        public Vec2 GetPosOnPlane(Vec3 pos)
        {
            _ = pos;
            return default;
        }

        public bool HasOverlap((int First, int Second) a, (int First, int Second) b)
        {
            return a.First == b.First
                   || a.First == b.Second
                   || a.Second == b.First
                   || a.Second == b.Second;
        }

        public void PutPointOnLines(Point p)
        {
            for (var i = 0; i < lines.Count; i++)
            {
                var l = lines[i];
                if (!l.IsPointOnLine(p.location3D))
                {
                    continue;
                }

                var reference = new ReferenceLine
                {
                    pointID = p.id,
                    lineID = l.id,
                    location = l.GetPosOnLine(p.location3D)
                };
                p.lines.Add(reference);
            }
        }

        // Normal is assumed to be normalised.
        public bool IsPointOnPlane(Vec3 pos)
        {
            var d = Vec3.Dot(normal, pos);
            return Utils.equals(distance, d, Eps.toleranceVectorEquality);
        }

        public PlaneBasis MakeBasis()
        {
            var origin = normal * distance;
            var up = normal;

            var worldUp = new Vec3(0, 1, 0);
            var worldRight = new Vec3(1, 0, 0);

            var normalIsUp = Utils.equals(up, worldUp, Eps.EPS_SMALL)
                             || Utils.equals(up * -1.0, worldUp, Eps.EPS_SMALL);
            var left = normalIsUp ? Vec3.Cross(up, worldRight) : Vec3.Cross(up, worldUp);
            var right = Vec3.Cross(left, up);

            return new PlaneBasis
            {
                origin = origin,
                up = Vec3.Normalize(up),
                left = Vec3.Normalize(left),
                right = Vec3.Normalize(right)
            };
        }
    }

    public sealed class Triangle
    {
        public int id;
        public int a;
        public int b;
        public int c;

        public void Flip()
        {
            var temp = a;
            a = b;
            b = temp;
        }

        public bool HasPoint(int p)
        {
            return a == p || b == p || c == p;
        }

        public bool IsNeighbour(Triangle t)
        {
            return HasPoint(t.a) || HasPoint(t.b) || HasPoint(t.c);
        }

        public bool SamePoints(Triangle other)
        {
            return other.HasPoint(a) && other.HasPoint(b) && other.HasPoint(c);
        }

        public int GetNotShared(Triangle other)
        {
            if (!other.HasPoint(a))
            {
                return a;
            }

            if (!other.HasPoint(b))
            {
                return b;
            }

            if (!other.HasPoint(c))
            {
                return c;
            }

            throw new InvalidOperationException("Missing common point in GetNotShared");
        }
    }

    public sealed class SegmentSet
    {
        public readonly List<(int First, int Second)> segments = new();
        public readonly List<Triangle> triangles = new();
        public readonly Dictionary<(int First, int Second), int> segmentCounts = new();
        public readonly List<int> irrelevantFaces = new();
        public readonly List<int> irrelevantFaces_toTest = new();

        public readonly Dictionary<int, List<(int First, int Second)>> planeSegments = new();
        public readonly Dictionary<int, Dictionary<(int First, int Second), int>> planeSegmentCounts = new();

        public void AddSegment(int planeId, int a, int b)
        {
            if (a == b)
            {
                if (Eps.messages)
                {
                    Console.WriteLine("a == b in AddSegment");
                }

                return;
            }

            var seg = a < b ? (a, b) : (b, a);

            segments.Add(seg);
            if (segmentCounts.TryGetValue(seg, out var count))
            {
                segmentCounts[seg] = count + 1;
            }
            else
            {
                segmentCounts[seg] = 1;
            }

            if (!planeSegments.TryGetValue(planeId, out var segList))
            {
                segList = new List<(int First, int Second)>();
                planeSegments[planeId] = segList;
            }

            segList.Add(seg);

            if (!planeSegmentCounts.TryGetValue(planeId, out var segCounts))
            {
                segCounts = new Dictionary<(int First, int Second), int>();
                planeSegmentCounts[planeId] = segCounts;
            }

            if (segCounts.TryGetValue(seg, out var planeCount))
            {
                segCounts[seg] = planeCount + 1;
            }
            else
            {
                segCounts[seg] = 1;
            }
        }

        public void AddFace(int planeId, int a, int b, int c)
        {
            AddSegment(planeId, a, b);
            AddSegment(planeId, b, c);
            AddSegment(planeId, c, a);

            var t = new Triangle
            {
                id = triangles.Count,
                a = a,
                b = b,
                c = c
            };
            triangles.Add(t);
        }

        public List<int> GetTrianglesWithPoint(int p)
        {
            var returnTriangles = new List<int>();
            for (var i = 0; i < triangles.Count; i++)
            {
                if (triangles[i].HasPoint(p))
                {
                    returnTriangles.Add(triangles[i].id);
                }
            }

            return returnTriangles;
        }

        public List<int> GetTrianglesWithEdge(int a, int b)
        {
            var returnTriangles = new List<int>();
            for (var i = 0; i < triangles.Count; i++)
            {
                if (triangles[i].HasPoint(a) && triangles[i].HasPoint(b))
                {
                    returnTriangles.Add(triangles[i].id);
                }
            }

            return returnTriangles;
        }

        public List<((int First, int Second) Edge, List<int> TriangleIds)> GetNeighbourTriangles(Triangle triangle)
        {
            var returnTriangles = new List<((int First, int Second), List<int>)>();
            returnTriangles.Add(((triangle.a, triangle.b), GetTrianglesWithEdge(triangle.a, triangle.b)));
            returnTriangles.Add(((triangle.b, triangle.c), GetTrianglesWithEdge(triangle.b, triangle.c)));
            returnTriangles.Add(((triangle.c, triangle.a), GetTrianglesWithEdge(triangle.c, triangle.a)));
            return returnTriangles;
        }

        public bool IsManifold()
        {
            foreach (var pair in segmentCounts)
            {
                if (pair.Value != 2)
                {
                    return false;
                }
            }

            return true;
        }

        public Dictionary<int, List<(int First, int Second)>> GetContourSegments()
        {
            var contours = new Dictionary<int, List<(int First, int Second)>>();

            foreach (var planePair in planeSegmentCounts)
            {
                var plane = planePair.Key;
                var counts = planePair.Value;
                foreach (var segmentCount in counts)
                {
                    if (segmentCount.Value != 1)
                    {
                        continue;
                    }

                    if (!contours.TryGetValue(plane, out var contour))
                    {
                        contour = new List<(int First, int Second)>();
                        contours[plane] = contour;
                    }

                    contour.Add(segmentCount.Key);
                }
            }

            return contours;
        }
    }

    public sealed class SharedPosition
    {
        private readonly record struct TriIndex(int A, int B, int C);

        public enum TriangleVsPoint
        {
            ABOVE,
            BELOW,
            ON
        }

        public readonly List<Point> points = new();
        public readonly List<Plane> planes = new();

        public readonly SegmentSet A = new();
        public readonly SegmentSet B = new();

        public Geometry? _linkedA;
        public Geometry? _linkedB;

        public readonly Geometry relevantA = new();
        public readonly Geometry relevantB = new();

        public BVH relevantBVHA = new();
        public BVH relevantBVHB = new();

        public readonly Dictionary<int, List<int>> planeToLines = new();
        public readonly Dictionary<int, List<int>> planeToPoints = new();

        public int FindUppermostTriangleId(Triangle @base, IReadOnlyList<int> triangleIds)
        {
            if (triangleIds.Count == 1)
            {
                return triangleIds[0];
            }

            var baseNorm = GetNormal(@base);
            var maxDotId = -1;
            var maxDot = -double.MaxValue;

            for (var i = 0; i < triangleIds.Count; i++)
            {
                var id = triangleIds[i];
                if (id == @base.id)
                {
                    continue;
                }

                var triNorm = GetNormal(A.triangles[id]);
                var other = A.triangles[id].GetNotShared(@base);
                var above = CalcTriPt(@base, other);

                var dot = (Vec3.Dot(baseNorm, triNorm) + 1.0) / 2.0;
                if (above == TriangleVsPoint.BELOW)
                {
                    dot -= 1.0;
                }
                else
                {
                    dot = 1.0 - dot;
                }

                if (dot > maxDot)
                {
                    maxDot = dot;
                    maxDotId = id;
                }
            }

            return maxDotId;
        }

        public int GetPointWithMaxY()
        {
            var max = -double.MaxValue;
            var pointId = 0;

            for (var i = 0; i < points.Count; i++)
            {
                if (points[i].location3D.Y > max)
                {
                    max = points[i].location3D.Y;
                    pointId = points[i].id;
                }
            }

            return pointId;
        }

        public TriangleVsPoint CalcTriPt(Triangle t, int point)
        {
            var norm = GetNormal(t);
            var dpt3d = points[point].location3D - points[t.a].location3D;
            var dot = Vec3.Dot(norm, dpt3d);

            if (Math.Abs(dot) < Eps.EPS_BIG)
            {
                return TriangleVsPoint.ON;
            }

            return dot > 0.0 ? TriangleVsPoint.ABOVE : TriangleVsPoint.BELOW;
        }

        public bool ShouldFlip(Triangle t, Triangle neighbour)
        {
            var normT = GetNormal(t);
            var normNB = GetNormal(neighbour);

            if (t.SamePoints(neighbour))
            {
                return Vec3.Dot(normT, normNB) < 1.0 - Eps.EPS_BIG;
            }

            var a = t.GetNotShared(neighbour);
            var e = neighbour.GetNotShared(t);

            var evsT = CalcTriPt(t, e);
            _ = a;
            var avsN = CalcTriPt(neighbour, e);

            if (evsT == TriangleVsPoint.ABOVE && avsN != TriangleVsPoint.ABOVE)
            {
                return true;
            }

            if (evsT == TriangleVsPoint.BELOW && avsN != TriangleVsPoint.BELOW)
            {
                return true;
            }

            if (evsT == TriangleVsPoint.ON && avsN != TriangleVsPoint.ON)
            {
                return true;
            }

            return false;
        }

        public Vec3 GetNormal(Triangle tri)
        {
            if (Utils.computeSafeNormal(
                    points[tri.a].location3D,
                    points[tri.b].location3D,
                    points[tri.c].location3D,
                    out var norm,
                    Eps.EPS_SMALL))
            {
                return norm;
            }

            return Vec3.Normalize(new Vec3(-1.0, -1.0, -1.0));
        }

        public SegmentSet GetSegSetA()
        {
            return A;
        }

        public int AddPoint(Vec3 newPoint)
        {
            for (var i = 0; i < points.Count; i++)
            {
                if (points[i].Matches(newPoint))
                {
                    return points[i].id;
                }
            }

            var p = new Point
            {
                id = points.Count,
                location3D = newPoint
            };
            points.Add(p);

            return p.id;
        }

        public int AddPlane(Vec3 normal, double d, uint refId)
        {
            for (var i = 0; i < planes.Count; i++)
            {
                if (planes[i].refPlane == refId || planes[i].IsEqualTo(normal, d))
                {
                    return planes[i].id;
                }
            }

            var p = new Plane
            {
                id = planes.Count,
                refPlane = (int)refId,
                normal = Vec3.Normalize(normal),
                distance = d
            };
            planes.Add(p);

            return p.id;
        }

        public void Construct(Geometry first, Geometry second, bool isUnion)
        {
            var boxA = first.GetAABB();
            var boxB = second.GetAABB();

            AddGeometry(first, second, boxB, isA: true, isUnion, 0);
            AddGeometry(second, first, boxA, isA: false, isUnion, (uint)first.Planes.Count);

            _linkedA = first;
            _linkedB = second;
        }

        public void AddGeometry(
            Geometry geom,
            Geometry secondGeom,
            AABB relevantBounds,
            bool isA,
            bool isUnion,
            uint offsetPlane)
        {
            for (var i = 0; i < geom.NumFaces; i++)
            {
                var f = geom.GetFace((int)i);
                var faceBox = geom.GetFaceBox((int)i);

                if (!faceBox.Intersects(relevantBounds))
                {
                    if (isA)
                    {
                        A.irrelevantFaces.Add((int)i);
                    }
                    else
                    {
                        B.irrelevantFaces.Add((int)i);
                    }

                    continue;
                }

                var contact = false;
                for (var j = 0; j < secondGeom.NumFaces; j++)
                {
                    var faceBox2 = secondGeom.GetFaceBox((int)j);
                    if (!faceBox.Intersects(faceBox2))
                    {
                        continue;
                    }

                    contact = true;
                    break;
                }

                if (!contact)
                {
                    if (isA)
                    {
                        A.irrelevantFaces_toTest.Add((int)i);
                    }
                    else if (isUnion || geom.NumFaces < 2000)
                    {
                        B.irrelevantFaces_toTest.Add((int)i);
                    }

                    continue;
                }

                var a = geom.GetPoint(f.I0);
                var b = geom.GetPoint(f.I1);
                var c = geom.GetPoint(f.I2);

                if (!Utils.computeSafeNormal(a, b, c, out var norm, Eps.EPS_SMALL))
                {
                    if (Eps.messages)
                    {
                        Console.WriteLine("Degenerate face in AddGeometry");
                    }

                    continue;
                }

                var sourcePlane = geom.Planes[(int)f.PId];
                var rs = Vec3.Dot(sourcePlane.Normal, norm);

                int planeId;
                if (rs < 0)
                {
                    planeId = AddPlane(sourcePlane.Normal * -1.0, -sourcePlane.Distance, f.PId + offsetPlane);
                }
                else
                {
                    planeId = AddPlane(sourcePlane.Normal, sourcePlane.Distance, f.PId + offsetPlane);
                }

                var ia = AddPoint(a);
                var ib = AddPoint(b);
                var ic = AddPoint(c);

                var da = Vec3.Dot(norm, a);
                var db = Vec3.Dot(norm, b);
                var dc = Vec3.Dot(norm, c);
                _ = da;
                _ = db;
                _ = dc;

                if (!planes[planeId].IsPointOnPlane(a) && Eps.messages)
                {
                    Console.WriteLine("unexpected point on plane in AddGeometry");
                }

                if (!planes[planeId].IsPointOnPlane(b) && Eps.messages)
                {
                    Console.WriteLine("unexpected point on plane in AddGeometry");
                }

                if (!planes[planeId].IsPointOnPlane(c) && Eps.messages)
                {
                    Console.WriteLine("unexpected point on plane in AddGeometry");
                }

                planes[planeId].AddPoint(a);
                planes[planeId].AddPoint(b);
                planes[planeId].AddPoint(c);

                if (isA)
                {
                    A.AddFace(planeId, ia, ib, ic);
                    if (planes[planeId].refPlane >= 0)
                    {
                        relevantA.AddFace(a, b, c, (uint)planes[planeId].refPlane);
                    }
                }
                else
                {
                    B.AddFace(planeId, ia, ib, ic);
                    if (planes[planeId].refPlane >= 0)
                    {
                        relevantB.AddFace(a, b, c, (uint)planes[planeId].refPlane);
                    }
                }
            }

            if (isA)
            {
                relevantBVHA = BVH.MakeBVH(relevantA);
            }
            else
            {
                relevantBVHB = BVH.MakeBVH(relevantB);
            }
        }

        public List<int> GetPointsOnPlane(Plane p)
        {
            if (!planeToPoints.TryGetValue(p.id, out var cp))
            {
                return new List<int>();
            }

            var sorted = new List<int>(cp);
            sorted.Sort();

            var dedup = new List<int>(sorted.Count);
            for (var i = 0; i < sorted.Count; i++)
            {
                if (i == 0 || sorted[i] != sorted[i - 1])
                {
                    dedup.Add(sorted[i]);
                }
            }

            return dedup;
        }

        // pair of start/end distance
        public List<(double Start, double End)> BuildSegments(IReadOnlyList<double> a, IReadOnlyList<double> b)
        {
            if (a.Count == 0 || b.Count == 0)
            {
                return new List<(double Start, double End)>();
            }

            var min = Math.Max(a[0], b[0]);
            var max = Math.Min(a[a.Count - 1], b[b.Count - 1]);

            var points = new List<double>();
            for (var i = 0; i < a.Count; i++)
            {
                var val = a[i];
                if (val >= min && val <= max)
                {
                    points.Add(val);
                }
            }

            for (var i = 0; i < b.Count; i++)
            {
                var val = b[i];
                if (val >= min && val <= max)
                {
                    points.Add(val);
                }
            }

            points.Sort((left, right) => left.CompareTo(right));

            var result = new List<(double Start, double End)>();
            result.Capacity = points.Count * 2;
            for (var i = 1; i < points.Count; i++)
            {
                result.Add((points[i - 1], points[i]));
            }

            var dedup = new List<(double Start, double End)>();
            for (var i = 0; i < result.Count; i++)
            {
                if (i == 0 || result[i] != result[i - 1])
                {
                    dedup.Add(result[i]);
                }
            }

            return dedup;
        }

        public List<(int First, int Second)> GetNonIntersectingSegments(Line l)
        {
            var pointsInOrder = new List<(int PointId, double Position)>();

            foreach (var segment in l.GetSegments())
            {
                if (!l.IsPointOnLine(points[segment.First].location3D) && Eps.messages)
                {
                    Console.WriteLine("point not on line in GetNonIntersectingSegments");
                }

                if (!l.IsPointOnLine(points[segment.Second].location3D) && Eps.messages)
                {
                    Console.WriteLine("point not on line in GetNonIntersectingSegments");
                }

                pointsInOrder.Add((segment.First, l.GetPosOnLine(points[segment.First].location3D)));
                pointsInOrder.Add((segment.Second, l.GetPosOnLine(points[segment.Second].location3D)));
            }

            pointsInOrder.Sort((left, right) => right.Position.CompareTo(left.Position));

            var segmentsWithoutIntersections = new List<(int First, int Second)>();
            if (pointsInOrder.Count == 0)
            {
                return segmentsWithoutIntersections;
            }

            var cur = pointsInOrder[0].PointId;
            for (var i = 1; i < pointsInOrder.Count; i++)
            {
                var next = pointsInOrder[i].PointId;
                if (cur != next)
                {
                    segmentsWithoutIntersections.Add((cur, next));
                    cur = next;
                }
            }

            return segmentsWithoutIntersections;
        }

        public void TriangulatePlane(Geometry geom, Plane p)
        {
            var pointsOnPlane = GetPointsOnPlane(p);
            var basis = p.MakeBasis();

            var pointToProjectedPoint = new Dictionary<int, int>(pointsOnPlane.Count);
            var projectedPointToPoint = new Dictionary<int, int>(pointsOnPlane.Count);
            var projectedPoints = new List<Vec2>(pointsOnPlane.Count);

            for (var i = 0; i < pointsOnPlane.Count; i++)
            {
                var pointId = pointsOnPlane[i];
                pointToProjectedPoint[pointId] = projectedPoints.Count;
                projectedPointToPoint[projectedPoints.Count] = pointId;
                projectedPoints.Add(basis.project(points[pointId].location3D));
            }

            var edges = new HashSet<(int First, int Second)>();
            var defaultEdges = new HashSet<(int First, int Second)>();

            for (var i = 0; i < p.lines.Count; i++)
            {
                var line = p.lines[i];
                var segments = GetNonIntersectingSegments(line);

                for (var j = 0; j < segments.Count; j++)
                {
                    var segment = segments[j];

                    if (!pointToProjectedPoint.ContainsKey(segment.First))
                    {
                        if (Eps.messages)
                        {
                            Console.WriteLine("unknown point in list, repairing in TriangulateLine");
                        }

                        pointToProjectedPoint[segment.First] = projectedPoints.Count;
                        projectedPointToPoint[projectedPoints.Count] = segment.First;
                        projectedPoints.Add(basis.project(points[segment.First].location3D));
                    }

                    if (!pointToProjectedPoint.ContainsKey(segment.Second))
                    {
                        if (Eps.messages)
                        {
                            Console.WriteLine("unknown point in list, repairing in TriangulateLine");
                        }

                        pointToProjectedPoint[segment.Second] = projectedPoints.Count;
                        projectedPointToPoint[projectedPoints.Count] = segment.Second;
                        projectedPoints.Add(basis.project(points[segment.Second].location3D));
                    }

                    var projectedIndexA = pointToProjectedPoint[segment.First];
                    var projectedIndexB = pointToProjectedPoint[segment.Second];

                    if (projectedIndexA == projectedIndexB)
                    {
                        continue;
                    }

                    defaultEdges.Add(segment);
                    edges.Add((projectedIndexA, projectedIndexB));
                }
            }

            // NOTE: C++ uses CDT triangulation. This fallback keeps workflow connected in C#.
            var triangles = TriangulateProjectedEdges(projectedPoints, edges);

            for (var i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];

                var pointIdA = projectedPointToPoint[tri.A];
                var pointIdB = projectedPointToPoint[tri.B];
                var pointIdC = projectedPointToPoint[tri.C];

                var ptA = points[pointIdA].location3D;
                var ptB = points[pointIdB].location3D;
                var ptC = points[pointIdC].location3D;

                var v1 = Vec3.Normalize(ptA - ptB);
                var v2 = Vec3.Normalize(ptA - ptC);
                var v3 = Vec3.Normalize(ptB - ptC);
                var rs1 = Vec3.Dot(v1, v2);
                var rs2 = Vec3.Dot(v2, v3);
                var rs3 = Vec3.Dot(v1, v3);

                if (Math.Abs(rs1) > 1 - Eps.toleranceThinTriangle
                    || Math.Abs(rs2) > 1 - Eps.toleranceThinTriangle
                    || Math.Abs(rs3) > 1 - Eps.toleranceThinTriangle)
                {
                    continue;
                }

                var triCenter = (ptA + ptB + ptC) / 3.0;
                var raydir = Utils.computeNormal(ptA, ptB, ptC);

                var posA = Utils.IsInsideMesh(triCenter, new Vec3(0, 0, 0), relevantA, relevantBVHA, raydir);
                var posB = Utils.IsInsideMesh(triCenter, new Vec3(0, 0, 0), relevantB, relevantBVHB, raydir);

                if (posA.Loc != MeshLocation.BOUNDARY && posB.Loc != MeshLocation.BOUNDARY)
                {
                    continue;
                }

                var t1 = projectedPoints[tri.A];
                var t2 = projectedPoints[tri.B];
                var t3 = projectedPoints[tri.C];

                var inside2d = Utils.IsInsideBoundary(t1, t2, t3, edges, projectedPoints);
                if (!inside2d)
                {
                    var postA = Utils.IsInsideMesh(triCenter, new Vec3(0, 0, 0), relevantA, relevantBVHA, raydir);
                    var postB = Utils.IsInsideMesh(triCenter, new Vec3(0, 0, 0), relevantB, relevantBVHB, raydir);

                    if (postA.Loc != MeshLocation.BOUNDARY && postB.Loc != MeshLocation.BOUNDARY)
                    {
                        continue;
                    }

                    var ptt = Mix(triCenter, ptA, Eps.triangleEvaluationFactor);
                    postA = Utils.IsInsideMesh(ptt, new Vec3(0, 0, 0), relevantA, relevantBVHA, raydir);
                    postB = Utils.IsInsideMesh(ptt, new Vec3(0, 0, 0), relevantB, relevantBVHB, raydir);
                    if (postA.Loc != MeshLocation.BOUNDARY && postB.Loc != MeshLocation.BOUNDARY)
                    {
                        continue;
                    }

                    ptt = Mix(triCenter, ptB, Eps.triangleEvaluationFactor);
                    postA = Utils.IsInsideMesh(ptt, new Vec3(0, 0, 0), relevantA, relevantBVHA, raydir);
                    postB = Utils.IsInsideMesh(ptt, new Vec3(0, 0, 0), relevantB, relevantBVHB, raydir);
                    if (postA.Loc != MeshLocation.BOUNDARY && postB.Loc != MeshLocation.BOUNDARY)
                    {
                        continue;
                    }

                    ptt = Mix(triCenter, ptC, Eps.triangleEvaluationFactor);
                    postA = Utils.IsInsideMesh(ptt, new Vec3(0, 0, 0), relevantA, relevantBVHA, raydir);
                    postB = Utils.IsInsideMesh(ptt, new Vec3(0, 0, 0), relevantB, relevantBVHB, raydir);
                    if (postA.Loc != MeshLocation.BOUNDARY && postB.Loc != MeshLocation.BOUNDARY)
                    {
                        continue;
                    }
                }

                if (p.refPlane < 0)
                {
                    continue;
                }

                // Keep C++ winding swap behavior.
                geom.AddFace(ptB, ptA, ptC, (uint)p.refPlane);
            }
        }

        public void AddRefPlaneToPoint(int point, int plane)
        {
            var reference = new ReferencePlane
            {
                pointID = point,
                planeID = plane
            };
            points[point].planes.Add(reference);

            if (!planeToPoints.TryGetValue(plane, out var planePoints))
            {
                planePoints = new List<int>();
                planeToPoints[plane] = planePoints;
            }

            planePoints.Add(point);
        }

        private static List<TriIndex> TriangulateProjectedEdges(IReadOnlyList<Vec2> projectedPoints, ISet<(int First, int Second)> edges)
        {
            var result = new List<TriIndex>();
            if (projectedPoints.Count < 3 || edges.Count == 0)
            {
                return result;
            }

            var loop = Utils.FindLargestEdgeLoop(projectedPoints, edges);
            var loopPoints = new List<int>(loop.loop);

            if (loopPoints.Count > 1 && loopPoints[0] == loopPoints[loopPoints.Count - 1])
            {
                loopPoints.RemoveAt(loopPoints.Count - 1);
            }

            if (loopPoints.Count < 3)
            {
                return result;
            }

            var root = loopPoints[0];
            for (var i = 1; i < loopPoints.Count - 1; i++)
            {
                result.Add(new TriIndex(root, loopPoints[i], loopPoints[i + 1]));
            }

            return result;
        }

        private static Vec3 Mix(Vec3 a, Vec3 b, double t)
        {
            return a * (1.0 - t) + b * t;
        }
    }

    public static partial class Utils
    {
        public static void AddSegments(Plane p, SharedPosition sp, Line templine, IReadOnlyList<(double Start, double End)> segments)
        {
            var isectLineId = p.AddLine(templine.origin, templine.direction);
            var isectLine = p.lines[isectLineId.LineId];

            if ((!p.IsPointOnPlane(isectLine.origin)
                 || !p.IsPointOnPlane(isectLine.origin + isectLine.direction * 100.0))
                && Eps.messages)
            {
                Console.WriteLine("Bad isect line in AddSegments");
            }

            for (var i = 0; i < segments.Count; i++)
            {
                var seg = segments[i];
                var pos = templine.GetPosOnLine(seg.Start);

                if (!p.aabb.Contains(pos) && Eps.messages)
                {
                    Console.WriteLine("making pos outside in AddSegments");
                }

                var ptA = sp.AddPoint(pos);
                var ptB = sp.AddPoint(templine.GetPosOnLine(seg.End));

                if (!p.aabb.Contains(sp.points[ptA].location3D) && Eps.messages)
                {
                    Console.WriteLine("bad points in AddSegments");
                }

                if (!p.aabb.Contains(sp.points[ptB].location3D) && Eps.messages)
                {
                    Console.WriteLine("bad points in AddSegments");
                }

                isectLine.AddPointToLine(isectLine.GetPosOnLine(sp.points[ptA].location3D), ptA);
                isectLine.AddPointToLine(isectLine.GetPosOnLine(sp.points[ptB].location3D), ptB);

                if (!p.IsPointOnPlane(sp.points[ptA].location3D) && Eps.messages)
                {
                    Console.WriteLine("bad point in AddSegments");
                }

                if (!p.IsPointOnPlane(sp.points[ptB].location3D) && Eps.messages)
                {
                    Console.WriteLine("bad point in AddSegments");
                }

                sp.AddRefPlaneToPoint(ptA, p.id);
                sp.AddRefPlaneToPoint(ptB, p.id);
            }
        }

        public static List<double> ComputeInitialIntersections(Plane p, SharedPosition sp, Line lineA)
        {
            var size = 1.0E+08;

            for (var i = 0; i < sp.points.Count; i++)
            {
                var d2 = DistanceSquared(lineA.origin, sp.points[i].location3D);
                size = Math.Max(size, d2);
            }

            size = Math.Sqrt(size);

            var astart = lineA.origin + lineA.direction * (size * 2);
            var aend = lineA.origin - lineA.direction * (size * 2);

            var distances = new List<double>(p.lines.Count);

            for (var i = 0; i < p.lines.Count; i++)
            {
                var line = p.lines[i];

                if (lineA.IsCollinear(line))
                {
                    continue;
                }

                foreach (var seg in line.GetSegments())
                {
                    var result = LineLineIntersection(
                        astart,
                        aend,
                        sp.points[seg.First].location3D,
                        sp.points[seg.Second].location3D);

                    if (result.distance >= Eps._TOLERANCE_PLANE_INTERSECTION)
                    {
                        continue;
                    }

                    if (!p.aabb.Contains(sp.points[seg.First].location3D) && Eps.messages)
                    {
                        Console.WriteLine("bad points in ComputeInitialIntersections");
                    }

                    if (!p.aabb.Contains(sp.points[seg.Second].location3D) && Eps.messages)
                    {
                        Console.WriteLine("bad points in ComputeInitialIntersections");
                    }

                    distances.Add(lineA.GetPosOnLine(result.point2));
                    var pt = lineA.GetPosOnLine(distances[distances.Count - 1]);

                    if (!p.aabb.Contains(result.point2) && Eps.messages)
                    {
                        Console.WriteLine("bad points in ComputeInitialIntersections");
                    }

                    if (!equals(pt, result.point2, Eps._TOLERANCE_PLANE_INTERSECTION) && Eps.messages)
                    {
                        Console.WriteLine("BAD POINT in ComputeInitialIntersections");
                    }
                }
            }

            distances.Sort((left, right) => left.CompareTo(right));

            var dedup = new List<double>(distances.Count);
            for (var i = 0; i < distances.Count; i++)
            {
                if (i == 0 || distances[i] != distances[i - 1])
                {
                    dedup.Add(distances[i]);
                }
            }

            return dedup;
        }

        public static void AddLineLineIntersections(Plane p, SharedPosition sp, Line lineA, Line lineB)
        {
            foreach (var segA in lineA.GetSegments())
            {
                foreach (var segB in lineB.GetSegments())
                {
                    if (p.HasOverlap(segA, segB))
                    {
                        continue;
                    }

                    var result = LineLineIntersection(
                        sp.points[segA.First].location3D,
                        sp.points[segA.Second].location3D,
                        sp.points[segB.First].location3D,
                        sp.points[segB.Second].location3D);

                    if (result.distance >= Eps.SCALED_EPS_BIG)
                    {
                        continue;
                    }

                    if (!p.aabb.Contains(result.point1))
                    {
                        if (Eps.messages)
                        {
                            Console.WriteLine("bad points in AddLineLineIntersections");
                        }

                        continue;
                    }

                    var point = sp.AddPoint(result.point1);

                    lineA.AddPointToLine(lineA.GetPosOnLine(sp.points[point].location3D), point);
                    lineB.AddPointToLine(lineB.GetPosOnLine(sp.points[point].location3D), point);

                    var refA = new ReferenceLine
                    {
                        pointID = point,
                        lineID = lineA.id,
                        location = lineA.GetPosOnLine(result.point1)
                    };
                    sp.points[point].lines.Add(refA);

                    for (var i = 0; i < lineA.planes.Count; i++)
                    {
                        sp.AddRefPlaneToPoint(point, lineA.planes[i].planeID);
                    }

                    var refB = new ReferenceLine
                    {
                        pointID = point,
                        lineID = lineB.id,
                        location = lineB.GetPosOnLine(result.point2)
                    };
                    sp.points[point].lines.Add(refB);

                    for (var i = 0; i < lineB.planes.Count; i++)
                    {
                        sp.AddRefPlaneToPoint(point, lineB.planes[i].planeID);
                    }
                }
            }
        }

        public static void AddLineLineIsects(Plane p, SharedPosition sp)
        {
            for (var lineAIndex = 0; lineAIndex < p.lines.Count; lineAIndex++)
            {
                for (var lineBIndex = lineAIndex + 1; lineBIndex < p.lines.Count; lineBIndex++)
                {
                    AddLineLineIntersections(p, sp, p.lines[lineAIndex], p.lines[lineBIndex]);
                }
            }
        }

        public static Geometry Normalize(Geometry first, Geometry second, SharedPosition sp, bool union)
        {
            var contoursA = sp.A.GetContourSegments();
            foreach (var contourPair in contoursA)
            {
                var planeId = contourPair.Key;
                var contours = contourPair.Value;
                var p = sp.planes[planeId];

                for (var i = 0; i < contours.Count; i++)
                {
                    var segment = contours[i];
                    p.AddLine(sp.points[segment.First], sp.points[segment.Second]);
                }
            }

            var contoursB = sp.B.GetContourSegments();
            foreach (var contourPair in contoursB)
            {
                var planeId = contourPair.Key;
                var contours = contourPair.Value;
                var p = sp.planes[planeId];

                for (var i = 0; i < contours.Count; i++)
                {
                    var segment = contours[i];
                    p.AddLine(sp.points[segment.First], sp.points[segment.Second]);
                }
            }

            // Put all points on lines/planes.
            for (var pointIndex = 0; pointIndex < sp.points.Count; pointIndex++)
            {
                var point = sp.points[pointIndex];
                for (var planeIndex = 0; planeIndex < sp.planes.Count; planeIndex++)
                {
                    var plane = sp.planes[planeIndex];
                    if (!plane.IsPointOnPlane(point.location3D))
                    {
                        continue;
                    }

                    sp.AddRefPlaneToPoint(point.id, plane.id);
                    plane.PutPointOnLines(point);
                }
            }

            for (var i = 0; i < sp.planes.Count; i++)
            {
                AddLineLineIsects(sp.planes[i], sp);
            }

            // Intersect planes.
            for (var planeAIndex = 0; planeAIndex < sp.planes.Count; planeAIndex++)
            {
                for (var planeBIndex = 0; planeBIndex < sp.planes.Count; planeBIndex++)
                {
                    var planeA = sp.planes[planeAIndex];
                    var planeB = sp.planes[planeBIndex];

                    if (!planeA.aabb.Intersects(planeB.aabb))
                    {
                        continue;
                    }

                    if (Math.Abs(Vec3.Dot(planeA.normal, planeB.normal)) > 1.0 - Eps.EPS_BIG)
                    {
                        continue;
                    }

                    var result = PlanePlaneIsect(planeA.normal, planeA.distance, planeB.normal, planeB.distance);

                    var intersectionLine = new Line
                    {
                        origin = result.pos,
                        direction = result.dir
                    };

                    if ((!planeA.IsPointOnPlane(intersectionLine.origin)
                         || !planeA.IsPointOnPlane(intersectionLine.origin + intersectionLine.direction * 1000.0))
                        && Eps.messages)
                    {
                        Console.WriteLine("Bad isect line in Normalize");
                    }

                    if ((!planeB.IsPointOnPlane(intersectionLine.origin)
                         || !planeB.IsPointOnPlane(intersectionLine.origin + intersectionLine.direction * 1000.0))
                        && Eps.messages)
                    {
                        Console.WriteLine("Bad isect line in Normalize");
                    }

                    var isectA = ComputeInitialIntersections(planeA, sp, intersectionLine);
                    var isectB = ComputeInitialIntersections(planeB, sp, intersectionLine);

                    var segments = sp.BuildSegments(isectA, isectB);
                    if (segments.Count == 0)
                    {
                        continue;
                    }

                    AddSegments(planeA, sp, intersectionLine, segments);
                    AddSegments(planeB, sp, intersectionLine, segments);
                }
            }

            for (var i = 0; i < sp.planes.Count; i++)
            {
                AddLineLineIsects(sp.planes[i], sp);
            }

            for (var pointIndex = 0; pointIndex < sp.points.Count; pointIndex++)
            {
                var point = sp.points[pointIndex];
                for (var planeIndex = 0; planeIndex < sp.planes.Count; planeIndex++)
                {
                    var plane = sp.planes[planeIndex];
                    if (plane.IsPointOnPlane(point.location3D))
                    {
                        sp.AddRefPlaneToPoint(point.id, plane.id);
                    }
                }
            }

            var geom = new Geometry();
            for (var i = 0; i < sp.planes.Count; i++)
            {
                sp.TriangulatePlane(geom, sp.planes[i]);
            }

            for (var i = 0; i < first.Planes.Count; i++)
            {
                var plane = first.Planes[i];
                geom.Planes.Add(new SimplePlane(plane.Normal, plane.Distance));
                geom.HasPlanes = true;
            }

            for (var i = 0; i < second.Planes.Count; i++)
            {
                var plane = second.Planes[i];
                geom.Planes.Add(new SimplePlane(plane.Normal, plane.Distance));
                geom.HasPlanes = true;
            }

            var offsetA = (uint)first.Planes.Count;

            if (sp._linkedA != null)
            {
                for (var i = 0; i < sp.A.irrelevantFaces_toTest.Count; i++)
                {
                    var faceIndex = sp.A.irrelevantFaces_toTest[i];
                    var f = sp._linkedA.GetFace(faceIndex);

                    var a = sp._linkedA.GetPoint(f.I0);
                    var b = sp._linkedA.GetPoint(f.I1);
                    var c = sp._linkedA.GetPoint(f.I2);

                    geom.AddFace(a, b, c, f.PId);
                }
            }

            if (sp._linkedB != null)
            {
                for (var i = 0; i < sp.B.irrelevantFaces_toTest.Count; i++)
                {
                    var faceIndex = sp.B.irrelevantFaces_toTest[i];
                    var f = sp._linkedB.GetFace(faceIndex);

                    var a = sp._linkedB.GetPoint(f.I0);
                    var b = sp._linkedB.GetPoint(f.I1);
                    var c = sp._linkedB.GetPoint(f.I2);

                    geom.AddFace(a, b, c, f.PId + offsetA);
                }
            }

            geom.Data = geom.NumFaces;

            if (sp._linkedA != null)
            {
                for (var i = 0; i < sp.A.irrelevantFaces.Count; i++)
                {
                    var faceIndex = sp.A.irrelevantFaces[i];
                    var f = sp._linkedA.GetFace(faceIndex);

                    var a = sp._linkedA.GetPoint(f.I0);
                    var b = sp._linkedA.GetPoint(f.I1);
                    var c = sp._linkedA.GetPoint(f.I2);

                    geom.AddFace(a, b, c, f.PId);
                }
            }

            if (union && sp._linkedB != null)
            {
                for (var i = 0; i < sp.B.irrelevantFaces.Count; i++)
                {
                    var faceIndex = sp.B.irrelevantFaces[i];
                    var f = sp._linkedB.GetFace(faceIndex);

                    var a = sp._linkedB.GetPoint(f.I0);
                    var b = sp._linkedB.GetPoint(f.I1);
                    var c = sp._linkedB.GetPoint(f.I2);

                    geom.AddFace(a, b, c, f.PId + offsetA);
                }
            }

            return geom;
        }

        private static double DistanceSquared(Vec3 a, Vec3 b)
        {
            var d = a - b;
            return Vec3.Dot(d, d);
        }
    }
}
