namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Profile
    {
        public ushort pType;
        public double width;
        public double depth;
        public double thickness;
        public double flangeThickness;
        public bool hasFillet;
        public double filletRadius;
        public double radius;
        public double slope;
        public ushort numSegments;
        public List<double> placement = new();
        public Curve profile = new();

        public void SetValues(
            ushort pType,
            double width,
            double depth,
            double webThickness,
            double flangeThickness,
            bool hasFillet,
            double filletRadius,
            double radius,
            double slope,
            ushort numSegments,
            List<double> placement)
        {
            this.pType = pType;
            this.width = width;
            this.depth = depth;
            this.thickness = webThickness;
            this.flangeThickness = flangeThickness;
            this.hasFillet = hasFillet;
            this.filletRadius = filletRadius;
            this.radius = radius;
            this.slope = slope;
            this.numSegments = numSegments;
            this.placement = new List<double>(placement);
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();
            var placementMatrix = BuildPlacementMatrix(placement);

            if (pType == 0)
            {
                profile = GetIShapedCurve(width, depth, thickness, flangeThickness, hasFillet, filletRadius, placementMatrix);
            }

            if (pType == 1)
            {
                profile = GetCShapedCurve(width, depth, thickness, flangeThickness, hasFillet, filletRadius, placementMatrix);
            }

            if (pType == 2)
            {
                profile = GetZShapedCurve(width, depth, thickness, flangeThickness, hasFillet, filletRadius, placementMatrix);
            }

            if (pType == 3)
            {
                profile = GetTShapedCurve(width, depth, thickness, hasFillet, filletRadius, radius, slope, placementMatrix);
            }

            if (pType == 4)
            {
                profile = GetLShapedCurve(width, depth, thickness, hasFillet, filletRadius, radius, slope, numSegments, placementMatrix);
            }

            if (pType == 5)
            {
                profile = GetUShapedCurve(width, depth, thickness, flangeThickness, filletRadius, radius, slope, placementMatrix);
            }

            for (var r = 0; r < profile.points.Count; r++)
            {
                buffers.AddPoint(profile.points[r]);
            }

            return buffers;
        }

        private static double[] BuildPlacementMatrix(IReadOnlyList<double> values)
        {
            if (values.Count < 16)
            {
                throw new ArgumentException("placement must contain at least 16 values.");
            }

            var matrix = new double[16];
            for (var i = 0; i < 16; i++)
            {
                matrix[i] = values[i];
            }

            return matrix;
        }

        // NOTE: These shape builders are defined in bim-geometry/utils.h in C++.
        // They are stubbed here until the corresponding utility conversions are added.
        private static Curve GetIShapedCurve(double width, double depth, double thickness, double flangeThickness, bool hasFillet, double filletRadius, double[] placement)
        {
            _ = width;
            _ = depth;
            _ = thickness;
            _ = flangeThickness;
            _ = hasFillet;
            _ = filletRadius;
            _ = placement;
            throw new NotImplementedException("GetIShapedCurve is not ported yet.");
        }

        private static Curve GetCShapedCurve(double width, double depth, double thickness, double flangeThickness, bool hasFillet, double filletRadius, double[] placement)
        {
            _ = width;
            _ = depth;
            _ = thickness;
            _ = flangeThickness;
            _ = hasFillet;
            _ = filletRadius;
            _ = placement;
            throw new NotImplementedException("GetCShapedCurve is not ported yet.");
        }

        private static Curve GetZShapedCurve(double width, double depth, double thickness, double flangeThickness, bool hasFillet, double filletRadius, double[] placement)
        {
            _ = width;
            _ = depth;
            _ = thickness;
            _ = flangeThickness;
            _ = hasFillet;
            _ = filletRadius;
            _ = placement;
            throw new NotImplementedException("GetZShapedCurve is not ported yet.");
        }

        private static Curve GetTShapedCurve(double width, double depth, double thickness, bool hasFillet, double filletRadius, double radius, double slope, double[] placement)
        {
            _ = width;
            _ = depth;
            _ = thickness;
            _ = hasFillet;
            _ = filletRadius;
            _ = radius;
            _ = slope;
            _ = placement;
            throw new NotImplementedException("GetTShapedCurve is not ported yet.");
        }

        private static Curve GetLShapedCurve(double width, double depth, double thickness, bool hasFillet, double filletRadius, double radius, double slope, ushort numSegments, double[] placement)
        {
            _ = width;
            _ = depth;
            _ = thickness;
            _ = hasFillet;
            _ = filletRadius;
            _ = radius;
            _ = slope;
            _ = numSegments;
            _ = placement;
            throw new NotImplementedException("GetLShapedCurve is not ported yet.");
        }

        private static Curve GetUShapedCurve(double width, double depth, double thickness, double flangeThickness, double filletRadius, double radius, double slope, double[] placement)
        {
            _ = width;
            _ = depth;
            _ = thickness;
            _ = flangeThickness;
            _ = filletRadius;
            _ = radius;
            _ = slope;
            _ = placement;
            throw new NotImplementedException("GetUShapedCurve is not ported yet.");
        }
    }
}
