using BazanCDE.Parsing.Utilities;
namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Sweep
    {
        public double scaling;
        public bool closed;
        public readonly List<Vec3> profilePoints = new();
        public readonly List<Vec3> directrix = new();
        public Vec3 initialDirectrixNormal = new(0, 0, 0);
        public bool rotate90;
        public bool optimize;

        public void SetValues(
            double scaling_,
            bool closed_,
            List<double> profilePoints_,
            List<double> directrix_,
            List<double> initialDirectrixNormal_,
            bool rotate90_,
            bool optimize_)
        {
            profilePoints.Clear();
            for (var i = 0; i + 2 < profilePoints_.Count; i += 3)
            {
                profilePoints.Add(new Vec3(profilePoints_[i], profilePoints_[i + 1], profilePoints_[i + 2]));
            }

            directrix.Clear();
            for (var i = 0; i + 2 < directrix_.Count; i += 3)
            {
                directrix.Add(new Vec3(directrix_[i], directrix_[i + 1], directrix_[i + 2]));
            }

            closed = closed_;
            scaling = scaling_;

            if (initialDirectrixNormal_.Count == 3)
            {
                initialDirectrixNormal = new Vec3(
                    initialDirectrixNormal_[0],
                    initialDirectrixNormal_[1],
                    initialDirectrixNormal_[2]);
            }

            rotate90 = rotate90_;
            optimize = optimize_;
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();

            var geom = Utils.SweepFunction(
                scaling,
                closed,
                profilePoints,
                directrix,
                initialDirectrixNormal,
                rotate90,
                optimize);

            for (var r = 0; r < geom.numFaces; r++)
            {
                var f = geom.GetFace((int)r);
                buffers.AddTri(geom.GetPoint(f.i0), geom.GetPoint(f.i1), geom.GetPoint(f.i2));
            }

            return buffers;
        }
    }
}
