using System.Collections.Generic;

namespace BazanCDE.Parsing.Operations.BooleanUtils
{
    public static partial class Utils
    {
        public static bool IsInsideBoundary(
            Vec2 t1,
            Vec2 t2,
            Vec2 t3,
            ISet<(int First, int Second)> edges,
            IReadOnlyList<Vec2> projectedPoints)
        {
            // Compute the centroid of the triangle.
            var centroid = (t1 + t2 + t3) / 3.0;

            // Use ray casting to determine if the centroid is inside the polygon.
            var crossings = 0;

            foreach (var edge in edges)
            {
                var p1 = projectedPoints[edge.First];
                var p2 = projectedPoints[edge.Second];

                // Check if the edge crosses the horizontal line at centroid.Y.
                if ((p1.Y > centroid.Y) != (p2.Y > centroid.Y))
                {
                    var xIntersection = p1.X + (p2.X - p1.X) * (centroid.Y - p1.Y) / (p2.Y - p1.Y);
                    if (xIntersection > centroid.X)
                    {
                        crossings++;
                    }
                }
            }

            // If crossings are odd, the point is inside the polygon.
            return (crossings % 2) == 1;
        }
    }
}
