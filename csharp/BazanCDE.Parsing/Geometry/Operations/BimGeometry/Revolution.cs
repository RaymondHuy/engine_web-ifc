namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Revolve
    {
        public double numRots;
        public double[] transform =
        {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        public double startDegrees;
        public double endDegrees;
        public readonly List<Vec3> profile = new();

        public void SetValues(
            List<double> profile_,
            List<double> transform_,
            double startDegrees_,
            double endDegrees_,
            uint numRots_)
        {
            numRots = numRots_;

            profile.Clear();
            for (var i = 0; i + 2 < profile_.Count; i += 3)
            {
                profile.Add(new Vec3(profile_[i], profile_[i + 1], profile_[i + 2]));
            }

            if (transform_.Count == 16)
            {
                transform = new[]
                {
                    transform_[0], transform_[1], transform_[2], transform_[3],
                    transform_[4], transform_[5], transform_[6], transform_[7],
                    transform_[8], transform_[9], transform_[10], transform_[11],
                    transform_[12], transform_[13], transform_[14], transform_[15]
                };
            }

            startDegrees = startDegrees_;
            endDegrees = endDegrees_;
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();
            var geom = Utils.Revolution(transform, startDegrees, endDegrees, profile, numRots);

            for (var r = 0; r < geom.NumFaces; r++)
            {
                var f = geom.GetFace((int)r);
                buffers.AddTri(geom.GetPoint(f.I0), geom.GetPoint(f.I1), geom.GetPoint(f.I2));
            }

            return buffers;
        }
    }
}
