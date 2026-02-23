namespace BazanCDE.Parsing.Geometry.Operations.BimGeometry
{
    public class Geometry
    {
        public bool hasPlanes = false;
        public uint numPoints = 0;
        public uint numFaces = 0;

        public List<float> fvertexData = new();
        public List<double> vertexData = new();
        public List<uint> indexData = new();
        public List<uint> planeData = new();
        public List<Plane> planes = new();

        public void AddPoint(Vec4 pt, Vec3 n)
        {
            AddPoint(pt.XYZ, n);
        }

        public AABB GetAABB()
        {
            var aabb = new AABB();

            for (uint i = 0; i < numPoints; i++)
            {
                aabb.min = Vec3.Min(aabb.min, GetPoint(i));
                aabb.max = Vec3.Max(aabb.max, GetPoint(i));
            }

            return aabb;
        }

        public void AddPoint(Vec3 pt, Vec3 n)
        {
            vertexData.Add(pt.X);
            vertexData.Add(pt.Y);
            vertexData.Add(pt.Z);

            vertexData.Add(n.X);
            vertexData.Add(n.Y);
            vertexData.Add(n.Z);

            numPoints += 1;
        }

        public Vec3 GetPoint(ulong index)
        {
            var o = (int)(index * Utils.VERTEX_FORMAT_SIZE_FLOATS);
            return new Vec3(
                vertexData[o + 0],
                vertexData[o + 1],
                vertexData[o + 2]);
        }

        public Vec3 GetPoint(uint index)
        {
            return GetPoint((ulong)index);
        }

        public Vec3 GetPoint(int index)
        {
            return GetPoint((ulong)index);
        }

        public void SetPoint(double x, double y, double z, ulong index)
        {
            var o = (int)(index * Utils.VERTEX_FORMAT_SIZE_FLOATS);
            vertexData[o + 0] = x;
            vertexData[o + 1] = y;
            vertexData[o + 2] = z;
        }

        public void SetPoint(double x, double y, double z, uint index)
        {
            SetPoint(x, y, z, (ulong)index);
        }

        public void SetPoint(double x, double y, double z, int index)
        {
            SetPoint(x, y, z, (ulong)index);
        }

        public Face GetFace(ulong index)
        {
            var f = new Face
            {
                i0 = (int)indexData[(int)(index * 3 + 0)],
                i1 = (int)indexData[(int)(index * 3 + 1)],
                i2 = (int)indexData[(int)(index * 3 + 2)],
                pId = (int)planeData[(int)index]
            };

            return f;
        }

        public Face GetFace(uint index)
        {
            return GetFace((ulong)index);
        }

        public Face GetFace(int index)
        {
            return GetFace((ulong)index);
        }

        public void AddFace(Vec3 a, Vec3 b, Vec3 c, uint pId = uint.MaxValue)
        {
            _ = Utils.areaOfTriangle(a, b, c);

            if (!Utils.computeSafeNormal(a, b, c, out var normal, Epsilons.toleranceAddFace))
            {
                return;
            }

            AddPoint(a, normal);
            AddPoint(b, normal);
            AddPoint(c, normal);

            AddFace(numPoints - 3, numPoints - 2, numPoints - 1, pId);
        }

        public void AddFace(uint a, uint b, uint c, uint pId = uint.MaxValue)
        {
            indexData.Add(a);
            indexData.Add(b);
            indexData.Add(c);

            _ = Utils.areaOfTriangle(GetPoint(a), GetPoint(b), GetPoint(c));

            numFaces++;
            planeData.Add(pId);
        }

        public ulong AddPlane(Vec3 normal, double d)
        {
            for (var i = 0; i < planes.Count; i++)
            {
                if (planes[i].IsEqualTo(normal, d))
                {
                    return planes[i].id;
                }
            }

            var p = new Plane
            {
                id = (ulong)planes.Count,
                normal = Vec3.Normalize(normal),
                distance = d
            };

            planes.Add(p);
            return p.id;
        }

        public void buildPlanes()
        {
            if (!hasPlanes)
            {
                var storedVertexData = new List<double>(vertexData);

                Vec3 GetStoredPoint(ulong index)
                {
                    var o = (int)(index * Utils.VERTEX_FORMAT_SIZE_FLOATS);
                    return new Vec3(
                        storedVertexData[o + 0],
                        storedVertexData[o + 1],
                        storedVertexData[o + 2]);
                }

                for (uint r = 0; r < Epsilons._PLANE_REFIT_ITERATIONS; r++)
                {
                    planes.Clear();
                    planeData.Clear();

                    for (ulong i = 0; i < numFaces; i++)
                    {
                        planeData.Add(uint.MaxValue);
                    }

                    var centroid = new Vec3(0, 0, 0);

                    for (ulong i = 0; i < numFaces; i++)
                    {
                        var f = GetFace(i);
                        var a = GetPoint(f.i0);
                        var b = GetPoint(f.i1);
                        var c = GetPoint(f.i2);

                        centroid = centroid + (a + b + c) / 3.0;
                    }

                    centroid /= numFaces;

                    for (ulong i = 0; i < numFaces; i++)
                    {
                        var f = GetFace(i);
                        var a = GetPoint(f.i0);
                        var b = GetPoint(f.i1);
                        var c = GetPoint(f.i2);

                        if (Utils.computeSafeNormal(a, b, c, out var norm, Epsilons.EPS_SMALL))
                        {
                            var da = Vec3.Dot(norm, a - centroid);
                            var db = Vec3.Dot(norm, b - centroid);
                            var dc = Vec3.Dot(norm, c - centroid);

                            var id = AddPlane(norm, (da + db + dc) / 3.0);
                            planeData[(int)i] = (uint)id;
                            hasPlanes = true;
                        }
                    }

                    for (ulong i = 0; i < numFaces; i++)
                    {
                        var f = GetFace(i);
                        var a = GetPoint(f.i0);
                        var b = GetPoint(f.i1);
                        var c = GetPoint(f.i2);

                        if (f.pId != -1)
                        {
                            var p = planes[f.pId];

                            var da = Vec3.Dot(p.normal, a - centroid);
                            var db = Vec3.Dot(p.normal, b - centroid);
                            var dc = Vec3.Dot(p.normal, c - centroid);

                            da = p.distance - da;
                            db = p.distance - db;
                            dc = p.distance - dc;

                            var va = a + p.normal * da;
                            var vb = b + p.normal * db;
                            var vc = c + p.normal * dc;

                            var dsa = GetStoredPoint((ulong)f.i0) - va;
                            var dsb = GetStoredPoint((ulong)f.i1) - vb;
                            var dsc = GetStoredPoint((ulong)f.i2) - vc;

                            var fa = dsa.Length() / Epsilons.reconstructTolerance;
                            var fb = dsb.Length() / Epsilons.reconstructTolerance;
                            var fc = dsc.Length() / Epsilons.reconstructTolerance;

                            if (fa > 1)
                            {
                                fa = dsa.Length() / fa;
                                dsa = Vec3.Normalize(dsa) * fa;
                                va += dsa;
                            }

                            if (fb > 1)
                            {
                                fb = dsb.Length() / fb;
                                dsb = Vec3.Normalize(dsb) * fb;
                                vb += dsb;
                            }

                            if (fc > 1)
                            {
                                fc = dsc.Length() / fc;
                                dsc = Vec3.Normalize(dsc) * fc;
                                vc += dsc;
                            }

                            SetPoint(va.X, va.Y, va.Z, f.i0);
                            SetPoint(vb.X, vb.Y, vb.Z, f.i1);
                            SetPoint(vc.X, vc.Y, vc.Z, f.i2);
                        }
                    }
                }

                for (ulong i = 0; i < numFaces; i++)
                {
                    var f = GetFace(i);

                    if (f.pId > -1)
                    {
                        var a = GetPoint(f.i0);
                        _ = GetPoint(f.i1);
                        _ = GetPoint(f.i2);

                        var da = Vec3.Dot(planes[f.pId].normal, a);
                        planes[f.pId].distance = da;
                    }
                }
            }

            // TODO: Remove unused planes
        }

        public void AddGeometry(Geometry geom)
        {
            for (uint i = 0; i < geom.numFaces; i++)
            {
                var f = geom.GetFace(i);
                var a = geom.GetPoint(f.i0);
                var b = geom.GetPoint(f.i1);
                var c = geom.GetPoint(f.i2);
                AddFace(a, b, c);
            }

            var planeDataOffset = geom.planes.Count;
            for (var i = 0; i < geom.planeData.Count; i++)
            {
                planeData.Add((uint)(planeDataOffset + geom.planeData[i]));
            }

            for (var i = 0; i < geom.planes.Count; i++)
            {
                planes.Add(geom.planes[i]);
            }
        }
    }
}
