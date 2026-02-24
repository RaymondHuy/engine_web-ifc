using BazanCDE.Parsing.Utilities;
namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Parabola : Curve
    {
        public ushort segments;
        public Vec2 startPoint;
        public double HorizontalLength;
        public double StartHeight;
        public double StartGradient;
        public double EndGradient;

        public void SetValues(
            ushort segments,
            double startPointX,
            double startPointY,
            double startPointZ,
            double horizontalLength,
            double startHeight,
            double startGradient,
            double endGradient)
        {
            _ = startPointZ; // C++ assigns dvec3 to dvec2, keeping XY.

            this.segments = segments;
            startPoint = new Vec2(startPointX, startPointY);
            HorizontalLength = horizontalLength;
            StartHeight = startHeight;
            StartGradient = startGradient;
            EndGradient = endGradient;
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();

            points.Clear();

            var points2D = Utils.SolveParabola(
                segments,
                startPoint,
                HorizontalLength,
                StartHeight,
                StartGradient,
                EndGradient);

            for (var i = 0; i < points2D.Count; i++)
            {
                points.Add(new Vec3(points2D[i].X, points2D[i].Y, 0));
            }

            for (var r = 0; r < points.Count; r++)
            {
                buffers.AddPoint(points[r]);
            }

            return buffers;
        }
    }
}
