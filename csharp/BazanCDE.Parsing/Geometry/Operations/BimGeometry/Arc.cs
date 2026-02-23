namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Arc : Curve
    {
        public float radiusX;
        public float radiusY;
        public int numSegments;
        public double[] placement =
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };
        public double startRad = 0;
        public double endRad = Utils.CONST_PI * 2;
        public bool swap = true;
        public bool normalToCenterEnding = false;

        public void SetValues(
            float radiusX,
            float radiusY,
            int numSegments,
            List<double> placement,
            double startRad = 0,
            double endRad = Utils.CONST_PI * 2,
            bool swap = true,
            bool normalToCenterEnding = false)
        {
            this.radiusX = radiusX;
            this.radiusY = radiusY;
            this.numSegments = numSegments;
            this.startRad = startRad;
            this.endRad = endRad;
            this.swap = swap;
            this.normalToCenterEnding = normalToCenterEnding;

            if (placement.Count != 9)
            {
                this.placement = new[]
                {
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                };
                return;
            }

            this.placement = new[]
            {
                placement[0], placement[1], placement[2],
                placement[3], placement[4], placement[5],
                placement[6], placement[7], placement[8]
            };
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();
            points.Clear();

            var ellipsePoints = Utils.GetEllipseCurve(
                radiusX,
                radiusY,
                numSegments,
                placement,
                startRad,
                endRad,
                swap,
                normalToCenterEnding).points;

            for (var r = 0; r < ellipsePoints.Count; r++)
            {
                buffers.AddPoint(ellipsePoints[r]);
            }

            return buffers;
        }
    }
}
