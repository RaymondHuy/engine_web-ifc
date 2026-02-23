namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Alignment : Curve
    {
        public ushort segments;
        public readonly List<Vec3> horizontal = new();
        public readonly List<Vec3> vertical = new();

        public void SetValues(List<double> horizontalValues, List<double> verticalValues)
        {
            horizontal.Clear();
            for (var i = 0; i + 2 < horizontalValues.Count; i += 3)
            {
                horizontal.Add(new Vec3(
                    horizontalValues[i],
                    horizontalValues[i + 1],
                    horizontalValues[i + 2]));
            }

            vertical.Clear();
            for (var i = 0; i + 2 < verticalValues.Count; i += 3)
            {
                vertical.Add(new Vec3(
                    verticalValues[i],
                    verticalValues[i + 1],
                    verticalValues[i + 2]));
            }
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();

            points.Clear();
            var points3D = Utils.Convert2DAlignmentsTo3D(horizontal, vertical);
            for (var i = 0; i < points3D.Count; i++)
            {
                points.Add(new Vec3(points3D[i].X, points3D[i].Y, points3D[i].Z));
            }

            for (var r = 0; r < points.Count; r++)
            {
                buffers.AddPoint(points[r]);
            }

            return buffers;
        }
    }
}
