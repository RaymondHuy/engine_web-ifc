namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class BooleanType
    {
        public int type;
        public string op = string.Empty;
        public Geometry geometry = new();
        public readonly List<Geometry> seconds = new();
        public readonly List<List<Vec3>> triangles = new();

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();
            var geom = Utils.BoolProcess(geometry, seconds, op);

            for (var r = 0; r < geom.numFaces; r++)
            {
                var f = geom.GetFace((int)r);
                buffers.AddTri(geom.GetPoint(f.i0), geom.GetPoint(f.i1), geom.GetPoint(f.i2));
            }

            return buffers;
        }

        public void SetValues(List<double> triangles_, string op_)
        {
            op = op_;
            geometry = new Geometry();

            for (var i = 0; i + 8 < triangles_.Count; i += 9)
            {
                geometry.AddFace(
                    new Vec3(triangles_[i], triangles_[i + 1], triangles_[i + 2]),
                    new Vec3(triangles_[i + 3], triangles_[i + 4], triangles_[i + 5]),
                    new Vec3(triangles_[i + 6], triangles_[i + 7], triangles_[i + 8]));
            }

            geometry.buildPlanes();
        }

        public void SetSecond(List<double> triangles_)
        {
            var newGeometry = new Geometry();
            for (var i = 0; i + 8 < triangles_.Count; i += 9)
            {
                newGeometry.AddFace(
                    new Vec3(triangles_[i], triangles_[i + 1], triangles_[i + 2]),
                    new Vec3(triangles_[i + 3], triangles_[i + 4], triangles_[i + 5]),
                    new Vec3(triangles_[i + 6], triangles_[i + 7], triangles_[i + 8]));
            }

            newGeometry.buildPlanes();
            seconds.Add(newGeometry);
        }

        public void clear()
        {
            geometry = new Geometry();
            seconds.Clear();
            op = string.Empty;
        }
    }
}
