using BazanCDE.Parsing.Utilities;
namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Clothoid : Curve
    {
        public ushort segments;
        public Vec2 startPoint;
        public double ifcStartDirection;
        public double StartRadiusOfCurvature;
        public double EndRadiusOfCurvature;
        public double SegmentLength;

        public void SetValues(
            ushort segments,
            double startPointX,
            double startPointY,
            double startPointZ,
            double ifcStartDirection,
            double startRadiusOfCurvature,
            double endRadiusOfCurvature,
            double segmentLength)
        {
            _ = startPointZ; // Kept for parity with C++ signature (dvec2 stores XY only).
            this.segments = segments;
            startPoint = new Vec2(startPointX, startPointY);
            this.ifcStartDirection = ifcStartDirection;
            StartRadiusOfCurvature = startRadiusOfCurvature;
            EndRadiusOfCurvature = endRadiusOfCurvature;
            SegmentLength = segmentLength;
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();
            points.Clear();

            var points2D = SolveClothoid(
                segments,
                startPoint,
                ifcStartDirection,
                StartRadiusOfCurvature,
                EndRadiusOfCurvature,
                SegmentLength);

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

        private static List<Vec2> SolveClothoid(
            ushort segments,
            Vec2 startPoint,
            double ifcStartDirection,
            double startRadiusOfCurvature,
            double endRadiusOfCurvature,
            double segmentLength)
        {
            var points = new List<Vec2>();

            var inverse = false;
            if (Math.Abs(startRadiusOfCurvature) > Math.Abs(endRadiusOfCurvature))
            {
                inverse = true;
            }

            var a = Math.Sqrt(Math.Abs(endRadiusOfCurvature - startRadiusOfCurvature) * segmentLength);
            var aPi = a * Math.Sqrt(Math.PI);
            var uMax = segmentLength / aPi;

            var s = a * uMax * Math.Sqrt(Math.PI);
            _ = (a * a * a) / (a * s); // radFin in C++ (unused)

            var vSin = 0.0;
            var vCos = 0.0;

            var directionX = new Vec2(
                Math.Cos(ifcStartDirection),
                Math.Sin(ifcStartDirection));
            var directionY = new Vec2(
                -Math.Sin(ifcStartDirection),
                Math.Cos(ifcStartDirection));

            if (endRadiusOfCurvature < 0 || startRadiusOfCurvature < 0)
            {
                directionY = new Vec2(-directionY.X, -directionY.Y);
            }

            if (inverse)
            {
                directionX = new Vec2(-directionX.X, -directionX.Y);
            }

            const double def = 2000;
            var dif = def / segments;
            var count = 0.0;
            var tram = uMax / (def - 1);
            var end = new Vec2(0, 0);
            var prev = new Vec2(0, 0);
            var endDir = new Vec2(0, 0);

            for (var c = 1.0; c < def + 1; c++)
            {
                prev = end;
                end = startPoint + aPi * (directionX * vCos + directionY * vSin);
                if (c == def || c == 1 || count >= dif)
                {
                    points.Add(end);
                    count = 0;
                }

                if (c == def)
                {
                    endDir = prev - end;
                }

                var val = c * tram;
                vSin += Math.Sin(Math.PI * ((a * val * val) / (2 * Math.Abs(a)))) * tram;
                vCos += Math.Cos(Math.PI * ((a * val * val) / (2 * Math.Abs(a)))) * tram;
                count++;
            }

            if (inverse)
            {
                directionX = new Vec2(-directionX.X, -directionX.Y);

                var newDirectionX = new Vec2(endDir.X, endDir.Y);
                var newDirectionY = new Vec2(-endDir.Y, endDir.X);

                if (endRadiusOfCurvature < 0 || startRadiusOfCurvature < 0)
                {
                    newDirectionY = new Vec2(-newDirectionY.X, -newDirectionY.Y);
                }

                newDirectionX = Vec2.Normalize(newDirectionX);
                newDirectionY = Vec2.Normalize(newDirectionY);

                for (var i = 0; i < points.Count; i++)
                {
                    var xx = points[i].X - end.X;
                    var yy = points[i].Y - end.Y;
                    var dx = xx * newDirectionX.X + yy * newDirectionX.Y;
                    var dy = xx * newDirectionY.X + yy * newDirectionY.Y;
                    var newDx = startPoint.X + directionX.X * dx + directionY.X * dy;
                    var newDy = startPoint.Y + directionX.Y * dx + directionY.Y * dy;
                    points[i] = new Vec2(newDx, newDy);
                }
            }

            return points;
        }
    }
}
