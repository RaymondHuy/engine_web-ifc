namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public sealed class Buffers
    {
        public readonly List<float> fvertexData = new();
        public readonly List<uint> indexData = new();

        public void AddPoint(Vec3 pt)
        {
            fvertexData.Add((float)pt.X);
            fvertexData.Add((float)pt.Y);
            fvertexData.Add((float)pt.Z);
        }

        public void AddTri(uint p1, uint p2, uint p3)
        {
            indexData.Add(p1);
            indexData.Add(p2);
            indexData.Add(p3);
        }

        public void AddTri(Vec3 p1, Vec3 p2, Vec3 p3)
        {
            AddPoint(p1);
            AddPoint(p2);
            AddPoint(p3);
            var id = fvertexData.Count / 3;
            AddTri((uint)(id - 3), (uint)(id - 2), (uint)(id - 1));
        }
    }
}
