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
                profile = Utils.GetIShapedCurve(width, depth, thickness, flangeThickness, hasFillet, filletRadius, placementMatrix);
            }

            if (pType == 1)
            {
                profile = Utils.GetCShapedCurve(width, depth, thickness, flangeThickness, hasFillet, filletRadius, placementMatrix);
            }

            if (pType == 2)
            {
                profile = Utils.GetZShapedCurve(width, depth, thickness, flangeThickness, filletRadius, radius, placementMatrix);
            }

            if (pType == 3)
            {
                profile = Utils.GetTShapedCurve(width, depth, thickness, hasFillet, filletRadius, radius, slope, placementMatrix);
            }

            if (pType == 4)
            {
                profile = Utils.GetLShapedCurve(width, depth, thickness, hasFillet, filletRadius, radius, slope, numSegments, placementMatrix);
            }

            if (pType == 5)
            {
                profile = Utils.GetUShapedCurve(width, depth, thickness, flangeThickness, filletRadius, radius, slope, placementMatrix);
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
    }
}
