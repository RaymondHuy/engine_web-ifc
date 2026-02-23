namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public static class Epsilons
    {
        public static double _TOLERANCE_SCALAR_EQUALITY = 1.0E-04;
        public static double _PLANE_REFIT_ITERATIONS = 1;
        public static double _BOOLEAN_UNION_THRESHOLD = 150;

        public const double EPS_TINY_CURVE = 1.0E-09;
        public const double EPS_NONZERO = 1.0E-20;
        public const double EPS_MINISCULE = 1.0E-12;
        public const double EPS_TINY = 1.0E-04;
        public const double EPS_SMALL = 1.0E-04;
        public const double EPS_BIG = 1.0E-04;
        public const double EPS_BIG2 = 1.0E-03;
        public const double SCALED_EPS_BIG = 1.0E-04;

        /*
            Constants used in function AddFace in geometry.h
        */
        public const double toleranceAddFace = 1.0E-10;
        public const double toleranceVectorEquality = 1.0E-04;

        /*
            Used in geometry.cpp
        */
        public const double reconstructTolerance = 1.0E-01;
    }
}
