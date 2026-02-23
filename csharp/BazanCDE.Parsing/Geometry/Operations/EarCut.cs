namespace BazanCDE.Parsing.Geometry.Operations
{
    public static class EarCut
    {
        public readonly record struct FlattenResult(List<double> Vertices, List<int> Holes, int Dimensions);

        private sealed class Node
        {
            public int I;
            public double X;
            public double Y;
            public Node? Prev;
            public Node? Next;
            public int Z;
            public Node? PrevZ;
            public Node? NextZ;
            public bool Steiner;
        }

        public static List<int> Earcut(IReadOnlyList<double> data, IReadOnlyList<int>? holeIndices = null, int dim = 2)
        {
            var hasHoles = holeIndices is { Count: > 0 };
            var outerLen = hasHoles ? holeIndices![0] * dim : data.Count;
            var outerNode = LinkedList(data, 0, outerLen, dim, clockwise: true);
            var triangles = new List<int>();

            if (outerNode == null || outerNode.Next == outerNode.Prev)
            {
                return triangles;
            }

            double minX = 0;
            double minY = 0;
            double invSize = 0;

            if (hasHoles)
            {
                outerNode = EliminateHoles(data, holeIndices!, outerNode, dim);
            }

            // If the shape is not too simple, use z-order hash; calculate polygon bbox.
            if (data.Count > 80 * dim)
            {
                minX = data[0];
                minY = data[1];
                var maxX = minX;
                var maxY = minY;

                for (var i = dim; i < outerLen; i += dim)
                {
                    var x = data[i];
                    var y = data[i + 1];
                    if (x < minX) minX = x;
                    if (y < minY) minY = y;
                    if (x > maxX) maxX = x;
                    if (y > maxY) maxY = y;
                }

                invSize = Math.Max(maxX - minX, maxY - minY);
                invSize = invSize != 0 ? 32767 / invSize : 0;
            }

            EarcutLinked(outerNode, triangles, dim, minX, minY, invSize, 0);
            return triangles;
        }

        public static double Deviation(IReadOnlyList<double> data, IReadOnlyList<int>? holeIndices, int dim, IReadOnlyList<int> triangles)
        {
            var hasHoles = holeIndices is { Count: > 0 };
            var outerLen = hasHoles ? holeIndices![0] * dim : data.Count;

            var polygonArea = Math.Abs(SignedArea(data, 0, outerLen, dim));
            if (hasHoles)
            {
                for (var i = 0; i < holeIndices!.Count; i++)
                {
                    var start = holeIndices[i] * dim;
                    var end = i < holeIndices.Count - 1 ? holeIndices[i + 1] * dim : data.Count;
                    polygonArea -= Math.Abs(SignedArea(data, start, end, dim));
                }
            }

            double trianglesArea = 0;
            for (var i = 0; i < triangles.Count; i += 3)
            {
                var a = triangles[i] * dim;
                var b = triangles[i + 1] * dim;
                var c = triangles[i + 2] * dim;
                trianglesArea += Math.Abs(
                    (data[a] - data[c]) * (data[b + 1] - data[a + 1]) -
                    (data[a] - data[b]) * (data[c + 1] - data[a + 1]));
            }

            if (polygonArea == 0 && trianglesArea == 0)
            {
                return 0;
            }

            return Math.Abs((trianglesArea - polygonArea) / polygonArea);
        }

        public static FlattenResult Flatten(IReadOnlyList<IReadOnlyList<IReadOnlyList<double>>> data)
        {
            var vertices = new List<double>();
            var holes = new List<int>();
            var dimensions = data[0][0].Count;
            var holeIndex = 0;
            var prevLen = 0;

            for (var r = 0; r < data.Count; r++)
            {
                var ring = data[r];
                for (var p = 0; p < ring.Count; p++)
                {
                    for (var d = 0; d < dimensions; d++)
                    {
                        vertices.Add(ring[p][d]);
                    }
                }

                if (prevLen != 0)
                {
                    holeIndex += prevLen;
                    holes.Add(holeIndex);
                }

                prevLen = ring.Count;
            }

            return new FlattenResult(vertices, holes, dimensions);
        }

        // create a circular doubly linked list from polygon points in the specified winding order
        private static Node? LinkedList(IReadOnlyList<double> data, int start, int end, int dim, bool clockwise)
        {
            Node? last = null;

            if (clockwise == (SignedArea(data, start, end, dim) > 0))
            {
                for (var i = start; i < end; i += dim)
                {
                    last = InsertNode(i / dim, data[i], data[i + 1], last);
                }
            }
            else
            {
                for (var i = end - dim; i >= start; i -= dim)
                {
                    last = InsertNode(i / dim, data[i], data[i + 1], last);
                }
            }

            if (last != null && Equals(last, last.Next!))
            {
                RemoveNode(last);
                last = last.Next;
            }

            return last;
        }

        // eliminate colinear or duplicate points
        private static Node? FilterPoints(Node? start, Node? end = null)
        {
            if (start == null)
            {
                return start;
            }

            end ??= start;

            var p = start;
            bool again;
            do
            {
                again = false;

                if (!p!.Steiner && (Equals(p, p.Next!) || Area(p.Prev!, p, p.Next!) == 0))
                {
                    RemoveNode(p);
                    p = end = p.Prev!;
                    if (p == p.Next)
                    {
                        break;
                    }

                    again = true;
                }
                else
                {
                    p = p.Next;
                }
            } while (again || p != end);

            return end;
        }

        // main ear slicing loop which triangulates a polygon (given as a linked list)
        private static void EarcutLinked(Node? ear, List<int> triangles, int dim, double minX, double minY, double invSize, int pass)
        {
            if (ear == null)
            {
                return;
            }

            // interlink polygon nodes in z-order
            if (pass == 0 && invSize != 0)
            {
                IndexCurve(ear, minX, minY, invSize);
            }

            var stop = ear;

            // iterate through ears, slicing them one by one
            while (ear.Prev != ear.Next)
            {
                var prev = ear.Prev!;
                var next = ear.Next!;

                if (invSize != 0 ? IsEarHashed(ear, minX, minY, invSize) : IsEar(ear))
                {
                    triangles.Add(prev.I);
                    triangles.Add(ear.I);
                    triangles.Add(next.I);

                    RemoveNode(ear);

                    // skipping the next vertex leads to less sliver triangles
                    ear = next.Next!;
                    stop = next.Next!;

                    continue;
                }

                ear = next;

                // if we looped through the whole remaining polygon and can't find any more ears
                if (ear == stop)
                {
                    // try filtering points and slicing again
                    if (pass == 0)
                    {
                        EarcutLinked(FilterPoints(ear), triangles, dim, minX, minY, invSize, 1);
                    }
                    // if this didn't work, try curing all small self-intersections locally
                    else if (pass == 1)
                    {
                        ear = CureLocalIntersections(FilterPoints(ear), triangles)!;
                        EarcutLinked(ear, triangles, dim, minX, minY, invSize, 2);
                    }
                    // as a last resort, try splitting the remaining polygon into two
                    else if (pass == 2)
                    {
                        SplitEarcut(ear, triangles, dim, minX, minY, invSize);
                    }

                    break;
                }
            }
        }

        // check whether a polygon node forms a valid ear with adjacent nodes
        private static bool IsEar(Node ear)
        {
            var a = ear.Prev!;
            var b = ear;
            var c = ear.Next!;

            if (Area(a, b, c) >= 0)
            {
                return false; // reflex, can't be an ear
            }

            var ax = a.X;
            var bx = b.X;
            var cx = c.X;
            var ay = a.Y;
            var by = b.Y;
            var cy = c.Y;

            // triangle bbox
            var x0 = Math.Min(ax, Math.Min(bx, cx));
            var y0 = Math.Min(ay, Math.Min(by, cy));
            var x1 = Math.Max(ax, Math.Max(bx, cx));
            var y1 = Math.Max(ay, Math.Max(by, cy));

            var p = c.Next!;
            while (p != a)
            {
                if (p.X >= x0 && p.X <= x1 && p.Y >= y0 && p.Y <= y1 &&
                    PointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, p.X, p.Y) &&
                    Area(p.Prev!, p, p.Next!) >= 0)
                {
                    return false;
                }

                p = p.Next!;
            }

            return true;
        }

        private static bool IsEarHashed(Node ear, double minX, double minY, double invSize)
        {
            var a = ear.Prev!;
            var b = ear;
            var c = ear.Next!;

            if (Area(a, b, c) >= 0)
            {
                return false; // reflex, can't be an ear
            }

            var ax = a.X;
            var bx = b.X;
            var cx = c.X;
            var ay = a.Y;
            var by = b.Y;
            var cy = c.Y;

            // triangle bbox
            var x0 = Math.Min(ax, Math.Min(bx, cx));
            var y0 = Math.Min(ay, Math.Min(by, cy));
            var x1 = Math.Max(ax, Math.Max(bx, cx));
            var y1 = Math.Max(ay, Math.Max(by, cy));

            // z-order range for the current triangle bbox
            var minZ = ZOrder(x0, y0, minX, minY, invSize);
            var maxZ = ZOrder(x1, y1, minX, minY, invSize);

            var p = ear.PrevZ;
            var n = ear.NextZ;

            // look for points inside the triangle in both directions
            while (p != null && p.Z >= minZ && n != null && n.Z <= maxZ)
            {
                if (p.X >= x0 && p.X <= x1 && p.Y >= y0 && p.Y <= y1 && p != a && p != c &&
                    PointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, p.X, p.Y) &&
                    Area(p.Prev!, p, p.Next!) >= 0)
                {
                    return false;
                }

                p = p.PrevZ;

                if (n.X >= x0 && n.X <= x1 && n.Y >= y0 && n.Y <= y1 && n != a && n != c &&
                    PointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, n.X, n.Y) &&
                    Area(n.Prev!, n, n.Next!) >= 0)
                {
                    return false;
                }

                n = n.NextZ;
            }

            // look for remaining points in decreasing z-order
            while (p != null && p.Z >= minZ)
            {
                if (p.X >= x0 && p.X <= x1 && p.Y >= y0 && p.Y <= y1 && p != a && p != c &&
                    PointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, p.X, p.Y) &&
                    Area(p.Prev!, p, p.Next!) >= 0)
                {
                    return false;
                }

                p = p.PrevZ;
            }

            // look for remaining points in increasing z-order
            while (n != null && n.Z <= maxZ)
            {
                if (n.X >= x0 && n.X <= x1 && n.Y >= y0 && n.Y <= y1 && n != a && n != c &&
                    PointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, n.X, n.Y) &&
                    Area(n.Prev!, n, n.Next!) >= 0)
                {
                    return false;
                }

                n = n.NextZ;
            }

            return true;
        }

        // go through all polygon nodes and cure small local self-intersections
        private static Node? CureLocalIntersections(Node? start, List<int> triangles)
        {
            var p = start;
            if (p == null)
            {
                return p;
            }

            do
            {
                var a = p!.Prev!;
                var b = p.Next!.Next!;

                if (!Equals(a, b) && Intersects(a, p, p.Next!, b) && LocallyInside(a, b) && LocallyInside(b, a))
                {
                    triangles.Add(a.I);
                    triangles.Add(p.I);
                    triangles.Add(b.I);

                    // remove two nodes involved
                    RemoveNode(p);
                    RemoveNode(p.Next!);

                    p = start = b;
                }

                p = p.Next;
            } while (p != start);

            return FilterPoints(p);
        }

        // try splitting polygon into two and triangulate them independently
        private static void SplitEarcut(Node start, List<int> triangles, int dim, double minX, double minY, double invSize)
        {
            // look for a valid diagonal that divides the polygon into two
            var a = start;
            do
            {
                var b = a.Next!.Next!;
                while (b != a.Prev)
                {
                    if (a.I != b.I && IsValidDiagonal(a, b))
                    {
                        // split the polygon in two by the diagonal
                        var c = SplitPolygon(a, b);

                        // filter collinear points around the cuts
                        a = FilterPoints(a, a.Next)!;
                        c = FilterPoints(c, c.Next)!;

                        // run earcut on each half
                        EarcutLinked(a, triangles, dim, minX, minY, invSize, 0);
                        EarcutLinked(c, triangles, dim, minX, minY, invSize, 0);
                        return;
                    }

                    b = b.Next!;
                }

                a = a.Next!;
            } while (a != start);
        }

        // link every hole into the outer loop, producing a single-ring polygon without holes
        private static Node EliminateHoles(IReadOnlyList<double> data, IReadOnlyList<int> holeIndices, Node outerNode, int dim)
        {
            var queue = new List<Node>();

            for (var i = 0; i < holeIndices.Count; i++)
            {
                var start = holeIndices[i] * dim;
                var end = i < holeIndices.Count - 1 ? holeIndices[i + 1] * dim : data.Count;
                var list = LinkedList(data, start, end, dim, clockwise: false);
                if (list == null)
                {
                    continue;
                }

                if (list == list.Next)
                {
                    list.Steiner = true;
                }

                queue.Add(GetLeftmost(list));
            }

            queue.Sort(CompareXYSlope);

            // process holes from left to right
            for (var i = 0; i < queue.Count; i++)
            {
                outerNode = EliminateHole(queue[i], outerNode);
            }

            return outerNode;
        }

        private static int CompareXYSlope(Node a, Node b)
        {
            var result = a.X - b.X;
            // when left-most points of two holes meet at a vertex, sort holes CCW
            if (result == 0)
            {
                result = a.Y - b.Y;
                if (result == 0)
                {
                    var aSlope = (a.Next!.Y - a.Y) / (a.Next.X - a.X);
                    var bSlope = (b.Next!.Y - b.Y) / (b.Next.X - b.X);
                    result = aSlope - bSlope;
                }
            }

            return result < 0 ? -1 : result > 0 ? 1 : 0;
        }

        // find a bridge between vertices that connects hole with an outer ring and link it
        private static Node EliminateHole(Node hole, Node outerNode)
        {
            var bridge = FindHoleBridge(hole, outerNode);
            if (bridge == null)
            {
                return outerNode;
            }

            var bridgeReverse = SplitPolygon(bridge, hole);

            // filter collinear points around the cuts
            FilterPoints(bridgeReverse, bridgeReverse.Next);
            return FilterPoints(bridge, bridge.Next)!;
        }

        // David Eberly's algorithm for finding a bridge between hole and outer polygon
        private static Node? FindHoleBridge(Node hole, Node outerNode)
        {
            var p = outerNode;
            var hx = hole.X;
            var hy = hole.Y;
            var qx = double.NegativeInfinity;
            Node? m = null;

            // find a segment intersected by a ray from hole's leftmost point to the left
            if (Equals(hole, p))
            {
                return p;
            }

            do
            {
                if (Equals(hole, p.Next!))
                {
                    return p.Next;
                }

                if (hy <= p.Y && hy >= p.Next!.Y && p.Next.Y != p.Y)
                {
                    var x = p.X + (hy - p.Y) * (p.Next.X - p.X) / (p.Next.Y - p.Y);
                    if (x <= hx && x > qx)
                    {
                        qx = x;
                        m = p.X < p.Next.X ? p : p.Next;
                        if (x == hx)
                        {
                            return m; // hole touches outer segment; pick leftmost endpoint
                        }
                    }
                }

                p = p.Next!;
            } while (p != outerNode);

            if (m == null)
            {
                return null;
            }

            // look for points inside the triangle of hole point, segment intersection and endpoint
            var stop = m;
            var mx = m.X;
            var my = m.Y;
            var tanMin = double.PositiveInfinity;

            p = m;

            do
            {
                if (hx >= p.X && p.X >= mx && hx != p.X &&
                    PointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p.X, p.Y))
                {
                    var tan = Math.Abs(hy - p.Y) / (hx - p.X); // tangential

                    if (LocallyInside(p, hole) &&
                        (tan < tanMin || (tan == tanMin && (p.X > m.X || (p.X == m.X && SectorContainsSector(m, p))))))
                    {
                        m = p;
                        tanMin = tan;
                    }
                }

                p = p.Next!;
            } while (p != stop);

            return m;
        }

        // whether sector in vertex m contains sector in vertex p in the same coordinates
        private static bool SectorContainsSector(Node m, Node p)
        {
            return Area(m.Prev!, m, p.Prev!) < 0 && Area(p.Next!, m, m.Next!) < 0;
        }

        // interlink polygon nodes in z-order
        private static void IndexCurve(Node start, double minX, double minY, double invSize)
        {
            var p = start;
            do
            {
                if (p.Z == 0)
                {
                    p.Z = ZOrder(p.X, p.Y, minX, minY, invSize);
                }

                p.PrevZ = p.Prev;
                p.NextZ = p.Next;
                p = p.Next!;
            } while (p != start);

            p.PrevZ!.NextZ = null;
            p.PrevZ = null;

            SortLinked(p);
        }

        // Simon Tatham's linked list merge sort
        private static Node? SortLinked(Node? list)
        {
            int numMerges;
            var inSize = 1;

            do
            {
                var p = list;
                Node? e;
                list = null;
                Node? tail = null;
                numMerges = 0;

                while (p != null)
                {
                    numMerges++;
                    var q = p;
                    var pSize = 0;
                    for (var i = 0; i < inSize; i++)
                    {
                        pSize++;
                        q = q.NextZ;
                        if (q == null)
                        {
                            break;
                        }
                    }

                    var qSize = inSize;

                    while (pSize > 0 || (qSize > 0 && q != null))
                    {
                        if (pSize != 0 && (qSize == 0 || q == null || p.Z <= q.Z))
                        {
                            e = p;
                            p = p.NextZ;
                            pSize--;
                        }
                        else
                        {
                            e = q;
                            q = q!.NextZ;
                            qSize--;
                        }

                        if (tail != null)
                        {
                            tail.NextZ = e;
                        }
                        else
                        {
                            list = e;
                        }

                        e!.PrevZ = tail;
                        tail = e;
                    }

                    p = q;
                }

                if (tail != null)
                {
                    tail.NextZ = null;
                }

                inSize *= 2;
            } while (numMerges > 1);

            return list;
        }

        // z-order of a point given coords and inverse of longer side of data bbox
        private static int ZOrder(double x, double y, double minX, double minY, double invSize)
        {
            // coords transformed into non-negative 15-bit integer range
            var xi = ((int)((x - minX) * invSize)) & 0x7fff;
            var yi = ((int)((y - minY) * invSize)) & 0x7fff;

            xi = (xi | (xi << 8)) & 0x00FF00FF;
            xi = (xi | (xi << 4)) & 0x0F0F0F0F;
            xi = (xi | (xi << 2)) & 0x33333333;
            xi = (xi | (xi << 1)) & 0x55555555;

            yi = (yi | (yi << 8)) & 0x00FF00FF;
            yi = (yi | (yi << 4)) & 0x0F0F0F0F;
            yi = (yi | (yi << 2)) & 0x33333333;
            yi = (yi | (yi << 1)) & 0x55555555;

            return xi | (yi << 1);
        }

        // find leftmost node of a polygon ring
        private static Node GetLeftmost(Node start)
        {
            var p = start;
            var leftmost = start;
            do
            {
                if (p.X < leftmost.X || (p.X == leftmost.X && p.Y < leftmost.Y))
                {
                    leftmost = p;
                }

                p = p.Next!;
            } while (p != start);

            return leftmost;
        }

        // check if a point lies within a convex triangle
        private static bool PointInTriangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py)
        {
            return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
                   (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
                   (bx - px) * (cy - py) >= (cx - px) * (by - py);
        }

        // check if point lies within convex triangle but false if equals first triangle point
        private static bool PointInTriangleExceptFirst(double ax, double ay, double bx, double by, double cx, double cy, double px, double py)
        {
            return !(ax == px && ay == py) && PointInTriangle(ax, ay, bx, by, cx, cy, px, py);
        }

        // check if diagonal between two polygon nodes is valid (lies in polygon interior)
        private static bool IsValidDiagonal(Node a, Node b)
        {
            return a.Next!.I != b.I && a.Prev!.I != b.I && !IntersectsPolygon(a, b) &&
                   ((LocallyInside(a, b) && LocallyInside(b, a) && MiddleInside(a, b) &&
                     (Area(a.Prev!, a, b.Prev!) != 0 || Area(a, b.Prev!, b) != 0)) ||
                    (Equals(a, b) && Area(a.Prev!, a, a.Next!) > 0 && Area(b.Prev!, b, b.Next!) > 0));
        }

        // signed area of a triangle
        private static double Area(Node p, Node q, Node r)
        {
            return (q.Y - p.Y) * (r.X - q.X) - (q.X - p.X) * (r.Y - q.Y);
        }

        // check if two points are equal
        private static bool Equals(Node p1, Node p2)
        {
            return p1.X == p2.X && p1.Y == p2.Y;
        }

        // check if two segments intersect
        private static bool Intersects(Node p1, Node q1, Node p2, Node q2)
        {
            var o1 = Sign(Area(p1, q1, p2));
            var o2 = Sign(Area(p1, q1, q2));
            var o3 = Sign(Area(p2, q2, p1));
            var o4 = Sign(Area(p2, q2, q1));

            if (o1 != o2 && o3 != o4)
            {
                return true; // general case
            }

            if (o1 == 0 && OnSegment(p1, p2, q1)) return true;
            if (o2 == 0 && OnSegment(p1, q2, q1)) return true;
            if (o3 == 0 && OnSegment(p2, p1, q2)) return true;
            if (o4 == 0 && OnSegment(p2, q1, q2)) return true;

            return false;
        }

        // for collinear points p,q,r, check if q lies on segment pr
        private static bool OnSegment(Node p, Node q, Node r)
        {
            return q.X <= Math.Max(p.X, r.X) && q.X >= Math.Min(p.X, r.X) &&
                   q.Y <= Math.Max(p.Y, r.Y) && q.Y >= Math.Min(p.Y, r.Y);
        }

        private static int Sign(double num)
        {
            return num > 0 ? 1 : num < 0 ? -1 : 0;
        }

        // check if a polygon diagonal intersects any polygon segments
        private static bool IntersectsPolygon(Node a, Node b)
        {
            var p = a;
            do
            {
                if (p.I != a.I && p.Next!.I != a.I && p.I != b.I && p.Next.I != b.I &&
                    Intersects(p, p.Next, a, b))
                {
                    return true;
                }

                p = p.Next;
            } while (p != a);

            return false;
        }

        // check if a polygon diagonal is locally inside the polygon
        private static bool LocallyInside(Node a, Node b)
        {
            return Area(a.Prev!, a, a.Next!) < 0
                ? Area(a, b, a.Next!) >= 0 && Area(a, a.Prev!, b) >= 0
                : Area(a, b, a.Prev!) < 0 || Area(a, a.Next!, b) < 0;
        }

        // check if the middle point of a polygon diagonal is inside the polygon
        private static bool MiddleInside(Node a, Node b)
        {
            var p = a;
            var inside = false;
            var px = (a.X + b.X) / 2;
            var py = (a.Y + b.Y) / 2;
            do
            {
                if (((p.Y > py) != (p.Next!.Y > py)) && p.Next.Y != p.Y &&
                    (px < (p.Next.X - p.X) * (py - p.Y) / (p.Next.Y - p.Y) + p.X))
                {
                    inside = !inside;
                }

                p = p.Next;
            } while (p != a);

            return inside;
        }

        // link two polygon vertices with a bridge
        private static Node SplitPolygon(Node a, Node b)
        {
            var a2 = CreateNode(a.I, a.X, a.Y);
            var b2 = CreateNode(b.I, b.X, b.Y);
            var an = a.Next!;
            var bp = b.Prev!;

            a.Next = b;
            b.Prev = a;

            a2.Next = an;
            an.Prev = a2;

            b2.Next = a2;
            a2.Prev = b2;

            bp.Next = b2;
            b2.Prev = bp;

            return b2;
        }

        // create a node and optionally link it with previous one
        private static Node InsertNode(int i, double x, double y, Node? last)
        {
            var p = CreateNode(i, x, y);

            if (last == null)
            {
                p.Prev = p;
                p.Next = p;
            }
            else
            {
                p.Next = last.Next;
                p.Prev = last;
                last.Next!.Prev = p;
                last.Next = p;
            }

            return p;
        }

        private static void RemoveNode(Node p)
        {
            p.Next!.Prev = p.Prev;
            p.Prev!.Next = p.Next;

            if (p.PrevZ != null) p.PrevZ.NextZ = p.NextZ;
            if (p.NextZ != null) p.NextZ.PrevZ = p.PrevZ;
        }

        private static Node CreateNode(int i, double x, double y)
        {
            return new Node
            {
                I = i,
                X = x,
                Y = y,
                Prev = null,
                Next = null,
                Z = 0,
                PrevZ = null,
                NextZ = null,
                Steiner = false
            };
        }

        public static double SignedArea(IReadOnlyList<double> data, int start, int end, int dim)
        {
            double sum = 0;

            for (int i = start, j = end - dim; i < end; i += dim)
            {
                sum += (data[j] - data[i]) * (data[i + 1] + data[j + 1]);
                j = i;
            }

            return sum;
        }
    }
}
