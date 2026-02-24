using BimGeom = BazanCDE.Parsing.Geometry.Operations.BimGeometry.Geometry;
using BimUtils = BazanCDE.Parsing.Geometry.Operations.BimGeometry.Utils;
using BazanCDE.Parsing.Geometry.Representation;
using BazanCDE.Parsing.Utilities;

namespace BazanCDE.Parsing.Geometry.Operations
{
    public class IfcGeometry : BimGeom
    {
        public bool isPolygon = false;
        public bool halfSpace = false;
        public List<IfcGeometry> part = new();
        public Vec3 halfSpaceX = new(1, 0, 0);
        public Vec3 halfSpaceY = new(0, 1, 0);
        public Vec3 halfSpaceZ = new(0, 0, 1);
        public Vec3 halfSpaceOrigin = new(0, 0, 0);
        public Vec3 normalizationCenter = new(0, 0, 0);
        public SweptDiskSolid sweptDiskSolid = new();
        private bool normalized = false;

        private void ReverseFace(uint index)
        {
            var o = (int)(index * 3);
            (indexData[o + 1], indexData[o + 2]) = (indexData[o + 2], indexData[o + 1]);
        }

        public void ReverseFaces()
        {
            for (uint i = 0; i < numFaces; i++)
            {
                ReverseFace(i);
            }
        }

        public void AddPart(IfcGeometry geom)
        {
            part.Add(geom);
        }

        public void AddPart(BimGeom geom)
        {
            var newPart = GeometryUtils.ToIfcGeometry(geom);
            part.Add(newPart);
        }

        public void AddGeometry(
            BimGeom geom,
            double[]? trans = null,
            double scx = 1,
            double scy = 1,
            double scz = 1,
            Vec3? origin = null)
        {
            var o = origin ?? new Vec3(0, 0, 0);
            var m = trans ?? GeometryUtils.Identity4;

            for (uint i = 0; i < geom.numFaces; i++)
            {
                var f = geom.GetFace(i);
                var a0 = geom.GetPoint(f.i0);
                var b0 = geom.GetPoint(f.i1);
                var c0 = geom.GetPoint(f.i2);

                var aScaled = new Vec3((a0.X - o.X) * scx + o.X, (a0.Y - o.Y) * scy + o.Y, (a0.Z - o.Z) * scz + o.Z);
                var bScaled = new Vec3((b0.X - o.X) * scx + o.X, (b0.Y - o.Y) * scy + o.Y, (b0.Z - o.Z) * scz + o.Z);
                var cScaled = new Vec3((c0.X - o.X) * scx + o.X, (c0.Y - o.Y) * scy + o.Y, (c0.Z - o.Z) * scz + o.Z);

                var a = GeometryUtils.TransformPoint4(m, aScaled);
                var b = GeometryUtils.TransformPoint4(m, bScaled);
                var c = GeometryUtils.TransformPoint4(m, cScaled);
                AddFace(a, b, c, f.pId < 0 ? uint.MaxValue : (uint)f.pId);
            }
        }

        public void MergeGeometry(BimGeom geom)
        {
            AddGeometry(geom);
        }

        public nint GetVertexData()
        {
            return nint.Zero;
        }

        public uint GetVertexDataSize()
        {
            return (uint)vertexData.Count;
        }

        public nint GetIndexData()
        {
            return nint.Zero;
        }

        public uint GetIndexDataSize()
        {
            return (uint)indexData.Count;
        }

        public SweptDiskSolid GetSweptDiskSolid()
        {
            return sweptDiskSolid;
        }

        public double[] Normalize()
        {
            normalized = true;
            return GeometryUtils.FlattenTransformation(GeometryUtils.Identity4);
        }
    }

    public static class GeometryUtils
    {
        private static readonly Random Random = new();

        internal static readonly double[] Identity4 =
        {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };

        public static void SetEpsilons(double toleranceScalarEquality, double planeRefitIterations, double booleanUnionThreshold)
        {
            BimUtils.SetEpsilons(toleranceScalarEquality, planeRefitIterations, booleanUnionThreshold);
        }

        public static double angleConversion(double angle, string angleUnits)
        {
            if (string.Equals(angleUnits, "RADIAN", StringComparison.Ordinal))
            {
                return angle;
            }

            return (angle / 360.0) * 2.0 * IfcRepresentationConstants.CONST_PI;
        }

        public static IfcGeometry ToIfcGeometry(BimGeom geom)
        {
            var ifcGeom = new IfcGeometry
            {
                numPoints = geom.numPoints,
                numFaces = geom.numFaces,
                hasPlanes = geom.hasPlanes
            };

            ifcGeom.fvertexData.AddRange(geom.fvertexData);
            ifcGeom.vertexData.AddRange(geom.vertexData);
            ifcGeom.indexData.AddRange(geom.indexData);
            ifcGeom.planeData.AddRange(geom.planeData);
            ifcGeom.planes.AddRange(geom.planes);

            if (ifcGeom.planeData.Count != ifcGeom.numFaces)
            {
                for (var i = ifcGeom.planeData.Count; i < ifcGeom.numFaces; i++)
                {
                    ifcGeom.planeData.Add(uint.MaxValue);
                }
            }

            return ifcGeom;
        }

        public static IfcGeometry Sweep(
            double scaling,
            bool closed,
            IfcProfile profile,
            IfcCurve directrix,
            Vec3? initialDirectrixNormal = null,
            bool rotate90 = false,
            bool optimize = true)
        {
            return ToIfcGeometry(
                BimUtils.SweepFunction(
                    scaling,
                    closed,
                    profile.curve.points,
                    directrix.points,
                    initialDirectrixNormal,
                    rotate90,
                    optimize));
        }

        public static IfcGeometry SweepCircular(
            double scaling,
            bool closed,
            IfcProfile profile,
            double radius,
            IfcCurve directrix,
            Vec3? initialDirectrixNormal = null,
            bool rotate90 = false)
        {
            var directrixVector = new List<Vec3>();
            for (var i = 0; i < directrix.points.Count; i++)
            {
                if (i < directrix.points.Count - 1)
                {
                    if ((directrix.points[i] - directrix.points[i + 1]).Length() > BazanCDE.Parsing.Geometry.Operations.BimGeometry.Epsilons.EPS_BIG2 * scaling)
                    {
                        directrixVector.Add(directrix.points[i]);
                    }
                }
                else
                {
                    directrixVector.Add(directrix.points[i]);
                }
            }

            var profileVector = new List<Vec3>(profile.curve.points);

            return ToIfcGeometry(
                BimUtils.SweepCircular(
                    scaling,
                    closed,
                    profileVector,
                    radius,
                    directrixVector,
                    initialDirectrixNormal,
                    rotate90));
        }

        public static bool computeSafeNormal(Vec3 v1, Vec3 v2, Vec3 v3, out Vec3 normal, double eps = 0)
        {
            return BimUtils.computeSafeNormal(v1, v2, v3, out normal, eps);
        }

        public static bool GetBasisFromCoplanarPoints(List<Vec3> points, out Vec3 v1, out Vec3 v2, out Vec3 v3)
        {
            v1 = points.Count > 0 ? points[0] : new Vec3(0, 0, 0);
            v2 = v1;
            v3 = v1;

            if (points.Count < 3)
            {
                return false;
            }

            foreach (var p in points)
            {
                if (v1 != p)
                {
                    v2 = p;
                    break;
                }
            }

            for (var i = 0; i < 4; i++)
            {
                var eps = IfcRepresentationConstants.EPS_SMALL;
                if (i == 0) eps = 100;
                if (i == 1) eps = 1;
                if (i == 2) eps = 0.01;
                if (i == 3) eps = 1e-03;

                foreach (var p in points)
                {
                    if (computeSafeNormal(v1, v2, p, out _, eps))
                    {
                        v3 = p;
                        return true;
                    }
                }
            }

            var d1 = 0.0;
            foreach (var p in points)
            {
                var d2 = (v1 - p).Length();
                if (d1 < d2)
                {
                    d1 = d2;
                    v2 = p;
                }
            }

            d1 = 0.0;
            foreach (var p in points)
            {
                var d2 = (v1 - p).Length();
                var d3 = (v2 - p).Length();
                if (d1 < d2 + d3)
                {
                    d1 = d2 + d3;
                    v3 = p;
                }
            }

            return computeSafeNormal(v1, v2, v3, out _, 1e-08);
        }

        public static void TriangulateBounds(IfcGeometry geometry, List<IfcBound3D> bounds, uint expressID)
        {
            if (bounds.Count == 1 && bounds[0].curve.points.Count == 3)
            {
                var c = bounds[0].curve;
                geometry.AddFace(c.points[0], c.points[1], c.points[2], uint.MaxValue);
                return;
            }

            if (bounds.Count == 0 || bounds[0].curve.points.Count < 3)
            {
                Console.WriteLine($"[TriangulateBounds()] bad bound {expressID}");
                return;
            }

            var offset = geometry.numPoints;

            if (bounds.Count > 1)
            {
                var outerIndex = -1;
                for (var i = 0; i < bounds.Count; i++)
                {
                    if (bounds[i].type == IfcBoundType.OUTERBOUND)
                    {
                        outerIndex = i;
                        break;
                    }
                }

                if (outerIndex == -1)
                {
                    Console.WriteLine($"[TriangulateBounds()] Expected outer bound! {expressID}");
                    return;
                }

                if (outerIndex != 0)
                {
                    (bounds[0], bounds[outerIndex]) = (bounds[outerIndex], bounds[0]);
                }
            }

            if (bounds[0].type != IfcBoundType.OUTERBOUND)
            {
                Console.WriteLine($"[TriangulateBounds()] Expected outer bound first! {expressID}");
            }

            if (!GetBasisFromCoplanarPoints(bounds[0].curve.points, out var v1, out var v2, out var v3))
            {
                Console.WriteLine($"[TriangulateBounds()] No basis found for brep! {expressID}");
                return;
            }

            var v12 = Vec3.Normalize(v3 - v2);
            var v13 = Vec3.Normalize(v1 - v2);
            var n = Vec3.Normalize(Vec3.Cross(v12, v13));
            v12 = Vec3.Cross(v13, n);

            var test = new IfcCurve();
            test.points.Capacity = bounds[0].curve.points.Count;
            for (var i = 0; i < bounds[0].curve.points.Count; i++)
            {
                var pt = bounds[0].curve.points[i];
                var pt2 = pt - v1;
                var proj = new Vec2(Vec3.Dot(pt2, v12), Vec3.Dot(pt2, v13));
                test.Add(proj);
            }

            if (!test.IsCCW())
            {
                n *= -1.0;
                (v12, v13) = (v13, v12);
            }

            var polygon = new List<List<Vec2>>(bounds.Count);
            for (var i = 0; i < bounds.Count; i++)
            {
                var ring = new List<Vec2>(bounds[i].curve.points.Count);
                foreach (var pt in bounds[i].curve.points)
                {
                    var pt2 = pt - v1;
                    ring.Add(new Vec2(Vec3.Dot(pt2, v12), Vec3.Dot(pt2, v13)));
                }

                polygon.Add(ring);
            }

            static bool PointInPolygon(IReadOnlyList<Vec2> poly, Vec2 p)
            {
                var inside = false;
                var n = poly.Count;
                if (n == 0)
                {
                    return false;
                }

                for (int i = 0, j = n - 1; i < n; j = i++)
                {
                    if ((poly[i].Y > p.Y) != (poly[j].Y > p.Y) &&
                        (p.X < poly[i].X + (poly[j].X - poly[i].X) * (p.Y - poly[i].Y) / (poly[j].Y - poly[i].Y + 1e-10)))
                    {
                        inside = !inside;
                    }
                }

                return inside;
            }

            var isValidOuter = true;
            for (var i = 1; i < polygon.Count; i++)
            {
                if (polygon[i].Count == 0)
                {
                    continue;
                }

                if (!PointInPolygon(polygon[0], polygon[i][0]))
                {
                    isValidOuter = false;
                    break;
                }
            }

            if (!isValidOuter)
            {
                Console.WriteLine($"[TriangulateBounds()] Labeled outer bound does not contain all inners! {expressID}");

                var trueOuterIndex = -1;
                for (var j = 0; j < polygon.Count; j++)
                {
                    var containsAll = true;
                    for (var k = 0; k < polygon.Count; k++)
                    {
                        if (k == j)
                        {
                            continue;
                        }

                        if (polygon[k].Count > 0 && !PointInPolygon(polygon[j], polygon[k][0]))
                        {
                            containsAll = false;
                            break;
                        }
                    }

                    if (containsAll)
                    {
                        trueOuterIndex = j;
                        break;
                    }
                }

                if (trueOuterIndex == -1)
                {
                    Console.WriteLine($"[TriangulateBounds()] No bound found that contains all others! {expressID}");
                    return;
                }

                if (trueOuterIndex != 0)
                {
                    (bounds[0], bounds[trueOuterIndex]) = (bounds[trueOuterIndex], bounds[0]);
                    (polygon[0], polygon[trueOuterIndex]) = (polygon[trueOuterIndex], polygon[0]);
                }

                if (!GetBasisFromCoplanarPoints(bounds[0].curve.points, out v1, out v2, out v3))
                {
                    Console.WriteLine($"[TriangulateBounds()] No basis found for true outer bound! {expressID}");
                    return;
                }

                v12 = Vec3.Normalize(v3 - v2);
                v13 = Vec3.Normalize(v1 - v2);
                n = Vec3.Normalize(Vec3.Cross(v12, v13));
                v12 = Vec3.Cross(v13, n);

                test = new IfcCurve();
                test.points.Capacity = bounds[0].curve.points.Count;
                foreach (var pt in bounds[0].curve.points)
                {
                    var pt2 = pt - v1;
                    test.Add(new Vec2(Vec3.Dot(pt2, v12), Vec3.Dot(pt2, v13)));
                }

                if (!test.IsCCW())
                {
                    n *= -1.0;
                    (v12, v13) = (v13, v12);
                }

                for (var i = 0; i < bounds.Count; i++)
                {
                    polygon[i].Clear();
                    polygon[i].Capacity = bounds[i].curve.points.Count;
                    foreach (var pt in bounds[i].curve.points)
                    {
                        var pt2 = pt - v1;
                        polygon[i].Add(new Vec2(Vec3.Dot(pt2, v12), Vec3.Dot(pt2, v13)));
                    }
                }
            }

            for (var i = 0; i < bounds.Count; i++)
            {
                foreach (var pt in bounds[i].curve.points)
                {
                    geometry.AddPoint(pt, n);
                }
            }

            var indices = TriangulateRings2D(polygon);
            for (var i = 0; i + 2 < indices.Count; i += 3)
            {
                geometry.AddFace(offset + indices[i], offset + indices[i + 1], offset + indices[i + 2], uint.MaxValue);
            }
        }

        public static IfcGeometry SectionedSurface(IfcCrossSections profiles, bool buildCaps)
        {
            var converted = new List<List<Vec3>>(profiles.curves.Count);
            for (var i = 0; i < profiles.curves.Count; i++)
            {
                converted.Add(new List<Vec3>(profiles.curves[i].points));
            }

            return ToIfcGeometry(BimUtils.SectionedSurface(converted, buildCaps, IfcRepresentationConstants.EPS_SMALL));
        }

        public static IfcGeometry Extrude(
            IfcProfile profile,
            Vec3 dir,
            double distance,
            Vec3? cuttingPlaneNormal = null,
            Vec3? cuttingPlanePos = null)
        {
            var profileCopy = new List<Vec3>(profile.curve.points);
            if (profileCopy.Count > 0 && (profileCopy[0] - profileCopy[profileCopy.Count - 1]).Length() > 1e-8)
            {
                profileCopy.Add(profileCopy[0]);
            }

            var profileVector = new List<List<Vec3>> { profileCopy };
            for (var i = 0; i < profile.holes.Count; i++)
            {
                profileVector.Add(new List<Vec3>(profile.holes[i].points));
            }

            return ToIfcGeometry(BimUtils.Extrude(profileVector, dir, distance, cuttingPlaneNormal, cuttingPlanePos));
        }

        public static IfcGeometry SweepFixedReference(
            double linearScalingFactor,
            bool closed,
            IfcProfile profile,
            IfcCurve directrix,
            Vec3 fixedReference)
        {
            _ = linearScalingFactor;

            var geom = new IfcGeometry();

            var refDir = Vec3.Normalize(fixedReference);
            var zAxis = new Vec3(0, 0, 1);
            var rotationAxis = Vec3.Cross(zAxis, refDir);
            var dot = Math.Clamp(Vec3.Dot(zAxis, refDir), -1.0, 1.0);
            var angle = Math.Acos(dot);
            var orientation = rotationAxis.Length() > IfcRepresentationConstants.EPS_SMALL
                ? RotationMatrixAxisAngle(angle, Vec3.Normalize(rotationAxis))
                : FlattenTransformation(Identity4);

            var profilePoints = profile.curve.points;
            var pathPoints = directrix.points;
            if (pathPoints.Count < 2 || profilePoints.Count < 2)
            {
                return geom;
            }

            var segments = closed ? pathPoints.Count : pathPoints.Count - 1;
            var startProfile = new List<Vec3>(profilePoints.Count);
            var endProfile = new List<Vec3>(profilePoints.Count);

            var startPos = pathPoints[0];
            for (var i = 0; i < profilePoints.Count; i++)
            {
                var transformed = TransformPoint4(orientation, profilePoints[i]);
                startProfile.Add(startPos + transformed);
            }

            for (uint i = 0; i < segments; i++)
            {
                var pos = pathPoints[(int)i];
                var nextPos = pathPoints[(int)((i + 1) % (uint)pathPoints.Count)];

                var currentProfile = new List<Vec3>(profilePoints.Count);
                for (var p = 0; p < profilePoints.Count; p++)
                {
                    currentProfile.Add(pos + TransformPoint4(orientation, profilePoints[p]));
                }

                List<Vec3> nextProfile;
                if (!closed || i < segments - 1)
                {
                    nextProfile = new List<Vec3>(profilePoints.Count);
                    for (var p = 0; p < profilePoints.Count; p++)
                    {
                        nextProfile.Add(nextPos + TransformPoint4(orientation, profilePoints[p]));
                    }
                }
                else
                {
                    nextProfile = startProfile;
                }

                if (i == segments - 1)
                {
                    endProfile = nextProfile;
                }

                for (var j = 0; j < profilePoints.Count; j++)
                {
                    var jNext = (j + 1) % profilePoints.Count;
                    geom.AddFace(currentProfile[j], nextProfile[j], nextProfile[jNext], uint.MaxValue);
                    geom.AddFace(currentProfile[j], nextProfile[jNext], currentProfile[jNext], uint.MaxValue);
                }
            }

            if (!closed)
            {
                var startCap = new IfcProfile
                {
                    type = profile.type,
                    isConvex = profile.isConvex,
                    isComposite = profile.isComposite
                };
                startCap.curve.points.AddRange(startProfile);
                geom.AddGeometry(Extrude(startCap, new Vec3(0, 0, -1), 0));

                var endCap = new IfcProfile
                {
                    type = profile.type,
                    isConvex = profile.isConvex,
                    isComposite = profile.isComposite
                };
                endCap.curve.points.AddRange(endProfile);
                geom.AddGeometry(Extrude(endCap, new Vec3(0, 0, 1), 0));
            }

            return geom;
        }

        public static double VectorToAngle2D(double x, double y)
        {
            return BimUtils.VectorToAngle(x, y);
        }

        public static bool MatrixFlipsTriangles(double[] mat)
        {
            return Determinant4(mat) < 0;
        }

        public static bool equals(Vec3 a, Vec3 b, double eps = 0)
        {
            return Math.Abs(a.X - b.X) <= eps
                && Math.Abs(a.Y - b.Y) <= eps
                && Math.Abs(a.Z - b.Z) <= eps;
        }

        public static bool equals(double a, double b, double eps = 0)
        {
            return Math.Abs(a - b) <= eps;
        }

        public static double areaOfTriangle(Vec3 a, Vec3 b, Vec3 c)
        {
            return BimUtils.areaOfTriangle(a, b, c);
        }

        public static double areaOfTriangle(Vec2 a, Vec2 b, Vec2 c)
        {
            return BimUtils.areaOfTriangle2D(a, b, c);
        }

        public static double RandomDouble(double lo, double hi)
        {
            return lo + Random.NextDouble() * (hi - lo);
        }

        public static Vec3? GetOriginRec(IfcComposedMesh mesh, Dictionary<uint, IfcGeometry> geometryMap, double[] mat)
        {
            var newMat = MatrixMultiply4(mat, mesh.transformation);

            if (geometryMap.TryGetValue(mesh.expressID, out var meshGeom) && meshGeom.numFaces > 0)
            {
                for (uint i = 0; i < meshGeom.numFaces; i++)
                {
                    var f = meshGeom.GetFace(i);
                    var a = TransformPoint4(newMat, meshGeom.GetPoint(f.i0));
                    return a;
                }
            }

            for (var i = 0; i < mesh.children.Count; i++)
            {
                var v = GetOriginRec(mesh.children[i], geometryMap, newMat);
                if (v.HasValue)
                {
                    return v;
                }
            }

            return null;
        }

        public static Vec3 GetOrigin(IfcComposedMesh mesh, Dictionary<uint, IfcGeometry> geometryMap)
        {
            var v = GetOriginRec(mesh, geometryMap, FlattenTransformation(Identity4));
            return v ?? new Vec3(0, 0, 0);
        }

        public static List<IfcGeometry> transformIfcGeometry(IfcGeometry sourceGeom, double[] matrix, bool transformationBreaksWinding)
        {
            var geomsTransformed = new List<IfcGeometry>();

            if (sourceGeom.part.Count > 0)
            {
                for (var i = 0; i < sourceGeom.part.Count; i++)
                {
                    var newMeshGeom = sourceGeom.part[i];
                    if (newMeshGeom.numFaces == 0)
                    {
                        continue;
                    }

                    var newGeom = new IfcGeometry
                    {
                        halfSpace = newMeshGeom.halfSpace
                    };

                    if (newGeom.halfSpace)
                    {
                        newGeom.halfSpaceOrigin = TransformPoint4(matrix, newMeshGeom.halfSpaceOrigin);
                        newGeom.halfSpaceX = TransformPoint4(matrix, newMeshGeom.halfSpaceX);
                        newGeom.halfSpaceY = TransformPoint4(matrix, newMeshGeom.halfSpaceY);
                        newGeom.halfSpaceZ = TransformPoint4(matrix, newMeshGeom.halfSpaceZ);
                    }

                    for (uint faceIndex = 0; faceIndex < newMeshGeom.numFaces; faceIndex++)
                    {
                        var f = newMeshGeom.GetFace(faceIndex);
                        var a = TransformPoint4(matrix, newMeshGeom.GetPoint(f.i0));
                        var b = TransformPoint4(matrix, newMeshGeom.GetPoint(f.i1));
                        var c = TransformPoint4(matrix, newMeshGeom.GetPoint(f.i2));

                        if (transformationBreaksWinding)
                        {
                            newGeom.AddFace(b, a, c, uint.MaxValue);
                        }
                        else
                        {
                            newGeom.AddFace(a, b, c, uint.MaxValue);
                        }
                    }

                    geomsTransformed.Add(newGeom);
                }
            }
            else
            {
                if (sourceGeom.numFaces > 0)
                {
                    var newGeom = new IfcGeometry
                    {
                        halfSpace = sourceGeom.halfSpace
                    };

                    if (newGeom.halfSpace)
                    {
                        newGeom.halfSpaceOrigin = TransformPoint4(matrix, sourceGeom.halfSpaceOrigin);
                        newGeom.halfSpaceX = TransformPoint4(matrix, sourceGeom.halfSpaceX);
                        newGeom.halfSpaceY = TransformPoint4(matrix, sourceGeom.halfSpaceY);
                        newGeom.halfSpaceZ = TransformPoint4(matrix, sourceGeom.halfSpaceZ);
                    }

                    for (uint faceIndex = 0; faceIndex < sourceGeom.numFaces; faceIndex++)
                    {
                        var f = sourceGeom.GetFace(faceIndex);
                        var a = TransformPoint4(matrix, sourceGeom.GetPoint(f.i0));
                        var b = TransformPoint4(matrix, sourceGeom.GetPoint(f.i1));
                        var c = TransformPoint4(matrix, sourceGeom.GetPoint(f.i2));

                        if (transformationBreaksWinding)
                        {
                            newGeom.AddFace(b, a, c, uint.MaxValue);
                        }
                        else
                        {
                            newGeom.AddFace(a, b, c, uint.MaxValue);
                        }
                    }

                    geomsTransformed.Add(newGeom);
                }
            }

            return geomsTransformed;
        }

        public static void flattenRecursive(IfcComposedMesh mesh, Dictionary<uint, IfcGeometry> geometryMap, List<IfcGeometry> geoms, double[] mat)
        {
            var newMat = MatrixMultiply4(mat, mesh.transformation);
            var transformationBreaksWinding = MatrixFlipsTriangles(newMat);

            if (geometryMap.TryGetValue(mesh.expressID, out var meshGeom))
            {
                geoms.AddRange(transformIfcGeometry(meshGeom, newMat, transformationBreaksWinding));
            }

            for (var i = 0; i < mesh.children.Count; i++)
            {
                flattenRecursive(mesh.children[i], geometryMap, geoms, newMat);
            }
        }

        public static List<IfcGeometry> flatten(IfcComposedMesh mesh, Dictionary<uint, IfcGeometry> geometryMap, double[]? mat = null)
        {
            var geoms = new List<IfcGeometry>();
            flattenRecursive(mesh, geometryMap, geoms, mat ?? FlattenTransformation(Identity4));
            return geoms;
        }

        public static List<IfcGeometry> flattenSolids(List<IfcGeometry> geoms)
        {
            var newGeoms = new List<IfcGeometry>();
            var newGeom = new IfcGeometry();

            for (var g = 0; g < geoms.Count; g++)
            {
                if (geoms[g].halfSpace)
                {
                    continue;
                }

                for (uint i = 0; i < geoms[g].numFaces; i++)
                {
                    var f = geoms[g].GetFace(i);
                    var a = geoms[g].GetPoint(f.i0);
                    var b = geoms[g].GetPoint(f.i1);
                    var c = geoms[g].GetPoint(f.i2);
                    newGeom.AddFace(a, b, c, uint.MaxValue);
                }
            }

            newGeoms.Add(newGeom);

            for (var g = 0; g < geoms.Count; g++)
            {
                if (geoms[g].halfSpace)
                {
                    newGeoms.Add(geoms[g]);
                }
            }

            return newGeoms;
        }

        public static double[] FlattenTransformation(double[] transformation)
        {
            var flat = new double[16];
            if (transformation.Length >= 16)
            {
                Array.Copy(transformation, flat, 16);
            }

            return flat;
        }

        public static bool notPresent(Vec3 pt, List<Vec3> points)
        {
            for (var i = 0; i < points.Count; i++)
            {
                if (pt.X == points[i].X && pt.Y == points[i].Y && pt.Z == points[i].Z)
                {
                    return false;
                }
            }

            return true;
        }

        internal static Vec3 TransformPoint4(double[] m, Vec3 v)
        {
            var x = m[0] * v.X + m[4] * v.Y + m[8] * v.Z + m[12];
            var y = m[1] * v.X + m[5] * v.Y + m[9] * v.Z + m[13];
            var z = m[2] * v.X + m[6] * v.Y + m[10] * v.Z + m[14];
            var w = m[3] * v.X + m[7] * v.Y + m[11] * v.Z + m[15];

            if (Math.Abs(w) > BazanCDE.Parsing.Geometry.Operations.BimGeometry.Epsilons.EPS_NONZERO &&
                Math.Abs(w - 1.0) > BazanCDE.Parsing.Geometry.Operations.BimGeometry.Epsilons.EPS_NONZERO)
            {
                return new Vec3(x / w, y / w, z / w);
            }

            return new Vec3(x, y, z);
        }

        private static double[] MatrixMultiply4(double[] a, double[] b)
        {
            var r = new double[16];
            for (var col = 0; col < 4; col++)
            {
                for (var row = 0; row < 4; row++)
                {
                    r[col * 4 + row] =
                        a[0 * 4 + row] * b[col * 4 + 0] +
                        a[1 * 4 + row] * b[col * 4 + 1] +
                        a[2 * 4 + row] * b[col * 4 + 2] +
                        a[3 * 4 + row] * b[col * 4 + 3];
                }
            }

            return r;
        }

        private static double[] RotationMatrixAxisAngle(double angle, Vec3 axis)
        {
            var x = axis.X;
            var y = axis.Y;
            var z = axis.Z;
            var c = Math.Cos(angle);
            var s = Math.Sin(angle);
            var t = 1.0 - c;

            return
            [
                t * x * x + c,       t * x * y + s * z,   t * x * z - s * y,   0,
                t * x * y - s * z,   t * y * y + c,       t * y * z + s * x,   0,
                t * x * z + s * y,   t * y * z - s * x,   t * z * z + c,       0,
                0,                   0,                   0,                   1
            ];
        }

        private static double Determinant4(double[] m)
        {
            var a00 = m[0];
            var a01 = m[4];
            var a02 = m[8];
            var a03 = m[12];
            var a10 = m[1];
            var a11 = m[5];
            var a12 = m[9];
            var a13 = m[13];
            var a20 = m[2];
            var a21 = m[6];
            var a22 = m[10];
            var a23 = m[14];
            var a30 = m[3];
            var a31 = m[7];
            var a32 = m[11];
            var a33 = m[15];

            var b00 = a00 * a11 - a01 * a10;
            var b01 = a00 * a12 - a02 * a10;
            var b02 = a00 * a13 - a03 * a10;
            var b03 = a01 * a12 - a02 * a11;
            var b04 = a01 * a13 - a03 * a11;
            var b05 = a02 * a13 - a03 * a12;
            var b06 = a20 * a31 - a21 * a30;
            var b07 = a20 * a32 - a22 * a30;
            var b08 = a20 * a33 - a23 * a30;
            var b09 = a21 * a32 - a22 * a31;
            var b10 = a21 * a33 - a23 * a31;
            var b11 = a22 * a33 - a23 * a32;

            return b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
        }

        private static List<uint> TriangulateRings2D(IReadOnlyList<IReadOnlyList<Vec2>> rings)
        {
            var data = new List<double>();
            var holes = new List<int>();

            var vertexCount = 0;
            for (var r = 0; r < rings.Count; r++)
            {
                if (r > 0)
                {
                    holes.Add(vertexCount);
                }

                var ring = rings[r];
                for (var i = 0; i < ring.Count; i++)
                {
                    data.Add(ring[i].X);
                    data.Add(ring[i].Y);
                }

                vertexCount += ring.Count;
            }

            var tri = EarCut.Earcut(data, holes.Count == 0 ? null : holes, 2);
            var indices = new List<uint>(tri.Count);
            for (var i = 0; i < tri.Count; i++)
            {
                indices.Add((uint)tri[i]);
            }

            return indices;
        }
    }
}
