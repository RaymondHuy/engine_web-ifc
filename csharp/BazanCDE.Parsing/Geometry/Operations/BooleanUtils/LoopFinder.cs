namespace BazanCDE.Parsing.Geometry.Operations.BooleanUtils
{
    public static partial class Utils
    {
        public sealed class Loop
        {
            public List<int> loop = new();

            public bool HasPoint(int p)
            {
                for (var i = 0; i < loop.Count; i++)
                {
                    if (loop[i] == p)
                    {
                        return true;
                    }
                }

                return false;
            }
        }

        // assume points vector is non overlapping, assume id is not in loop
        public static bool IsPointInsideLoop(IReadOnlyList<Vec2> points, Loop loop, Vec2 pt)
        {
            var line = new Vec2(10000, 0);
            var distances = new List<double>();

            for (var i = 0; i < loop.loop.Count; i++)
            {
                var prev = loop.loop[i];
                var cur = loop.loop[(i + 1) % loop.loop.Count];

                var a = points[prev];
                var b = points[cur];

                var dir = Vec2.Normalize(a - b);
                var colinear = Math.Abs(Vec2.Dot(dir, line)) > (1 - Eps.EPS_BIG);
                if (colinear)
                {
                    continue;
                }

                var result = doLineSegmentsIntersect(pt, pt + line, a, b, Eps.EPS_BIG);
                if (result.isect)
                {
                    distances.Add(result.dist);
                }
            }

            distances.Sort((a, b) => b.CompareTo(a));

            var count = 0;
            var curDist = -1.0;
            for (var i = 0; i < distances.Count; i++)
            {
                if (Math.Abs(distances[i] - curDist) > Eps.EPS_BIG)
                {
                    count++;
                }

                curDist = distances[i];
            }

            return count % 2 == 1;
        }

        public static bool AInsideB(IReadOnlyList<Vec2> points, Loop a, Loop b)
        {
            int? nonSharedPoint = null; // point of A not in B

            for (var i = 0; i < a.loop.Count; i++)
            {
                if (!b.HasPoint(a.loop[i]))
                {
                    nonSharedPoint = a.loop[i];
                }
            }

            // all points of A are on B, A is inside of B.
            if (!nonSharedPoint.HasValue)
            {
                return true;
            }

            var pt = points[nonSharedPoint.Value];

            // found point index that is in A but not in B
            return IsPointInsideLoop(points, b, pt);
        }

        public static Loop FindOuterLoop(IReadOnlyList<Vec2> points, ISet<(int First, int Second)> edges, bool forward)
        {
            var result = new Loop();
            if (edges.Count == 0)
            {
                return result;
            }

            var connections = new Dictionary<int, SortedSet<int>>();
            foreach (var edge in edges)
            {
                if (!connections.TryGetValue(edge.First, out var firstSet))
                {
                    firstSet = new SortedSet<int>();
                    connections[edge.First] = firstSet;
                }

                if (!connections.TryGetValue(edge.Second, out var secondSet))
                {
                    secondSet = new SortedSet<int>();
                    connections[edge.Second] = secondSet;
                }

                firstSet.Add(edge.Second);
                secondSet.Add(edge.First);
            }

            // keep walking right. If we find a ccw loop, we're good.
            var firstEdge = edges.First();
            var cur = firstEdge.First;
            var prev = firstEdge.Second;
            if (forward)
            {
                (cur, prev) = (prev, cur);
            }

            var start = prev;
            var loop = new List<int>();
            var visited = new bool[points.Count];

            while (true)
            {
                if (cur < 0 || cur >= visited.Length || prev < 0 || prev >= points.Count)
                {
                    loop.Clear();
                    return result;
                }

                if (visited[cur])
                {
                    if (Eps.messages)
                    {
                        Console.WriteLine("Loop in findLoop ... how ironic");
                    }

                    return new Loop();
                }

                visited[cur] = true;
                loop.Add(cur);
                if (cur == start)
                {
                    // end of loop
                    break;
                }

                if (!connections.TryGetValue(cur, out var neighbors))
                {
                    if (Eps.messages)
                    {
                        Console.WriteLine("Found vert without neighbours other than the origin of this search!");
                    }

                    loop.Clear();
                    return result;
                }

                // copy neighbors and remove backward/self
                var nbs = new SortedSet<int>(neighbors);
                nbs.Remove(prev);
                nbs.Remove(cur);

                if (nbs.Count == 0)
                {
                    // broken poly
                    if (Eps.messages)
                    {
                        Console.WriteLine("Found vert without neighbours other than the origin of this search!");
                    }

                    loop.Clear();
                    return result;
                }

                // find right-hand neighbour with narrowest "turn"
                var maxSign = -double.MaxValue;
                var maxNB = 0;

                var a = points[prev];
                var b = points[cur];

                foreach (var nb in nbs)
                {
                    var p = points[nb];
                    var sign = ComparableAngle(p, a, b);

                    if (sign > maxSign)
                    {
                        maxSign = sign;
                        maxNB = nb;
                    }
                }

                // assign maxNB to cur
                prev = cur;
                cur = maxNB;
            }

            result.loop = loop;
            return result;
        }

        public static Loop FindLargestEdgeLoop(IReadOnlyList<Vec2> points, ISet<(int First, int Second)> edges)
        {
            var l1 = FindOuterLoop(points, edges, false);
            var l2 = FindOuterLoop(points, edges, true);

            if (AInsideB(points, l1, l2))
            {
                return l2;
            }

            return l1;
        }

    }
}
