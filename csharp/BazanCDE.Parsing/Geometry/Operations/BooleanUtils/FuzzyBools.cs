namespace BazanCDE.Parsing.Geometry.Operations.BooleanUtils
{
    public static partial class Utils
    {
        public static void SetEpsilons(
            double tolerancePlaneIntersection,
            double tolerancePlaneDeviation,
            double toleranceBackDeviationDistance,
            double toleranceInsideOutsidePerimeter,
            double toleranceBoundingBox,
            double boolStatus)
        {
            Eps._TOLERANCE_PLANE_INTERSECTION = tolerancePlaneIntersection;
            Eps._TOLERANCE_PLANE_DEVIATION = tolerancePlaneDeviation;
            Eps._TOLERANCE_BACK_DEVIATION_DISTANCE = toleranceBackDeviationDistance;
            Eps._TOLERANCE_INSIDE_OUTSIDE_PERIMETER = toleranceInsideOutsidePerimeter;
            Eps._TOLERANCE_BOUNDING_BOX = toleranceBoundingBox;
            Eps._BOOLSTATUS = boolStatus;
        }

        public static Geometry Subtract(Geometry a, Geometry b)
        {
            var sp = new SharedPosition();
            sp.Construct(a, b, false);

            var bvh1 = BVH.MakeBVH(a);
            var bvh2 = BVH.MakeBVH(b);

            var geom = Normalize(a, b, sp, false);

            return clipSubtract(geom, bvh1, bvh2);
        }

        public static Geometry Union(Geometry a, Geometry b)
        {
            var sp = new SharedPosition();
            sp.Construct(a, b, true);

            var bvh1 = BVH.MakeBVH(a);
            var bvh2 = BVH.MakeBVH(b);

            var geom = Normalize(a, b, sp, true);

            return clipJoin(geom, bvh1, bvh2);
        }
    }
}
