namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Extrusion
    {
        public bool cap;
        public double len;
        public Vec3 dir = new(0, 0, 0);
        public Vec3 cuttingPlanePos = new(0, 0, 0);
        public Vec3 cuttingPlaneNormal = new(0, 0, 0);
        public readonly List<Vec3> profile = new();
        public readonly List<List<Vec3>> holes = new();

        public void SetValues(
            List<double> profile_,
            List<double> dir_,
            double len_,
            List<double> cuttingPlaneNormal_,
            List<double> cuttingPlanePos_,
            bool cap_)
        {
            profile.Clear();
            for (var i = 0; i + 2 < profile_.Count; i += 3)
            {
                profile.Add(new Vec3(profile_[i], profile_[i + 1], profile_[i + 2]));
            }

            if (dir_.Count == 3)
            {
                dir = new Vec3(dir_[0], dir_[1], dir_[2]);
            }

            len = len_;
            cap = cap_;

            if (cuttingPlanePos_.Count >= 3)
            {
                cuttingPlanePos = new Vec3(cuttingPlanePos_[0], cuttingPlanePos_[1], cuttingPlanePos_[2]);
            }

            if (cuttingPlaneNormal_.Count >= 3)
            {
                cuttingPlaneNormal = new Vec3(cuttingPlaneNormal_[0], cuttingPlaneNormal_[1], cuttingPlaneNormal_[2]);
            }
        }

        public void SetHoles(List<double> hole_)
        {
            var hole = new List<Vec3>();
            for (var i = 0; i + 2 < hole_.Count; i += 3)
            {
                hole.Add(new Vec3(hole_[i], hole_[i + 1], hole_[i + 2]));
            }

            holes.Add(hole);
        }

        public void ClearHoles()
        {
            holes.Clear();
        }

        public Buffers GetBuffers()
        {
            var buffers = new Buffers();

            BazanCDE.Parsing.Geometry.Operations.BooleanUtils.Geometry geom;
            if (!cap)
            {
                geom = Utils.Extrude(profile, dir, len);
            }
            else
            {
                var profileNumber = 1 + holes.Count;
                var profiles = new List<List<Vec3>>(profileNumber);

                for (var i = 0; i < profileNumber; i++)
                {
                    profiles.Add(new List<Vec3>());
                }

                for (var i = 0; i < profile.Count; i++)
                {
                    profiles[0].Add(new Vec3(profile[i].X, profile[i].Y, profile[i].Z));
                }

                for (var i = 0; i < holes.Count; i++)
                {
                    for (var j = 0; j < holes[i].Count; j++)
                    {
                        profiles[i + 1].Add(new Vec3(holes[i][j].X, holes[i][j].Y, holes[i][j].Z));
                    }
                }

                geom = Utils.Extrude(profiles, dir, len, cuttingPlaneNormal, cuttingPlanePos);
            }

            for (var r = 0; r < geom.NumFaces; r++)
            {
                var f = geom.GetFace((int)r);
                buffers.AddTri(geom.GetPoint(f.I0), geom.GetPoint(f.I1), geom.GetPoint(f.I2));
            }

            return buffers;
        }
    }
}
