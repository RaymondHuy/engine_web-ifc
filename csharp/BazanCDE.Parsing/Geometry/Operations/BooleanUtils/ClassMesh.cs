
namespace BazanCDE.Parsing.Operations.BooleanUtils
{
    public static partial class Utils
    {
        public static void doubleClipSingleMesh(Geometry mesh, BVH bvh1, BVH bvh2, Geometry result)
        {
            var boundingList = new List<(int FaceIndex, AABB Box)>();

            for (var i = 0; i < mesh.Planes.Count; i++)
            {
                result.HasPlanes = true;
                result.Planes.Add(mesh.Planes[i]);
            }

            var bvh1Ptr = bvh1.Ptr ?? throw new InvalidOperationException("bvh1.Ptr is null.");
            var bvh2Ptr = bvh2.Ptr ?? throw new InvalidOperationException("bvh2.Ptr is null.");

            for (var i = 0u; i < mesh.Data; i++)
            {
                var doIt = false;
                var tri = mesh.GetFace((int)i);
                var a = mesh.GetPoint(tri.I0);
                var b = mesh.GetPoint(tri.I1);
                var c = mesh.GetPoint(tri.I2);

                var aabb = mesh.GetFaceBox((int)i);

                if (!aabb.Intersects(bvh2.Box))
                {
                    // when subtracting, if box is outside the second operand, it's guaranteed to remain.
                }
                else if (!aabb.Intersects(bvh1.Box))
                {
                    // when subtracting, if box is outside the first operand, it won't remain ever.
                }

                var doNext = true;

                for (var pairIndex = 0; pairIndex < boundingList.Count; pairIndex++)
                {
                    var pair = boundingList[pairIndex];
                    if (!aabb.Intersects(pair.Box))
                    {
                        continue;
                    }

                    var triTemp = mesh.GetFace(pair.FaceIndex);

                    var at = mesh.GetPoint(triTemp.I0);
                    var bt = mesh.GetPoint(triTemp.I1);
                    var ct = mesh.GetPoint(triTemp.I2);

                    if ((equals(at, a, Eps.EPS_MINISCULE) && equals(bt, b, Eps.EPS_MINISCULE) && equals(ct, c, Eps.EPS_MINISCULE))
                        || (equals(at, b, Eps.EPS_MINISCULE) && equals(bt, c, Eps.EPS_MINISCULE) && equals(ct, a, Eps.EPS_MINISCULE))
                        || (equals(at, c, Eps.EPS_MINISCULE) && equals(bt, a, Eps.EPS_MINISCULE) && equals(ct, b, Eps.EPS_MINISCULE)))
                    {
                        doNext = false;
                        break;
                    }
                }

                if (!doNext)
                {
                    continue;
                }

                boundingList.Add(((int)i, aabb));

                var n = computeNormal(a, b, c);
                var triCenter = (a + b * 1.02 + c * 1.03) * (1.0 / 3.05); // Using true centroid could cause issues (#540)
                var raydir = computeNormal(a, b, c);

                var isInside1Loc = IsInsideMesh(triCenter, n, bvh1Ptr, bvh1, raydir);
                var isInside2Loc = IsInsideMesh(triCenter, n, bvh2Ptr, bvh2, raydir);

                var extraDir1 = Vec3.Normalize(raydir + new Vec3(0.02, 0.01, 0.04));
                var extraDir2 = Vec3.Normalize(raydir + new Vec3(0.20, -0.1, 0.40));

                var isInside1LocB = IsInsideMesh(triCenter, n, bvh1Ptr, bvh1, extraDir1);
                var isInside2LocB = IsInsideMesh(triCenter, n, bvh2Ptr, bvh2, extraDir1);

                if (isInside1Loc.Loc != isInside1LocB.Loc)
                {
                    var isInside1LocC = IsInsideMesh(triCenter, n, bvh1Ptr, bvh1, extraDir2);
                    if (isInside1LocC.Loc == isInside1LocB.Loc)
                    {
                        isInside1Loc = isInside1LocB;
                    }
                    else if (isInside1LocB.Loc != isInside1LocC.Loc && isInside1Loc.Loc != isInside1LocC.Loc)
                    {
                        isInside1Loc = isInside1LocB;
                    }
                }

                if (isInside2Loc.Loc != isInside2LocB.Loc)
                {
                    var isInside2LocC = IsInsideMesh(triCenter, n, bvh2Ptr, bvh2, extraDir2);
                    if (isInside2LocC.Loc == isInside2LocB.Loc)
                    {
                        isInside2Loc = isInside2LocB;
                    }
                    else if (isInside2LocB.Loc != isInside2LocC.Loc && isInside2Loc.Loc != isInside2LocC.Loc)
                    {
                        isInside2Loc = isInside2LocB;
                    }
                }

                var isInside1 = isInside1Loc.Loc;
                var isInside2 = isInside2Loc.Loc;

                if (isInside1 == MeshLocation.OUTSIDE && isInside2 == MeshLocation.OUTSIDE)
                {
                    // both outside, no dice
                }

                if (isInside1 != MeshLocation.BOUNDARY && isInside2 != MeshLocation.BOUNDARY)
                {
                    // neither boundary, no dice
                }
                else if (isInside1 == MeshLocation.BOUNDARY && isInside2 == MeshLocation.BOUNDARY)
                {
                    // both boundary, no dice if normals are same direction
                    var dot = Vec3.Dot(isInside1Loc.Normal, isInside2Loc.Normal);
                    if (dot < 0)
                    {
                        result.AddFace(a, b, c, tri.PId);
                        doIt = true;
                    }
                }
                else
                {
                    if (isInside2 == MeshLocation.INSIDE || isInside1 == MeshLocation.OUTSIDE)
                    {
                        // inside 2 with subtract => don't include
                        // outside 1 with subtract => don't include
                    }
                    else
                    {
                        // boundary or outside 2, and boundary or inside 1 => keep
                        if (isInside2 == MeshLocation.BOUNDARY && isInside1 == MeshLocation.INSIDE)
                        {
                            if (Vec3.Dot(n, isInside2Loc.Normal) < 0)
                            {
                                result.AddFace(a, b, c, tri.PId);
                                doIt = true;
                            }
                            else
                            {
                                result.AddFace(b, a, c, tri.PId);
                                doIt = true;
                            }
                        }
                        else if (isInside1 == MeshLocation.BOUNDARY)
                        {
                            if (Vec3.Dot(n, isInside1Loc.Normal) < 0)
                            {
                                result.AddFace(b, a, c, tri.PId);
                                doIt = true;
                            }
                            else
                            {
                                result.AddFace(a, b, c, tri.PId);
                                doIt = true;
                            }
                        }
                        else
                        {
                            result.AddFace(a, b, c, tri.PId);
                            doIt = true;
                        }
                    }
                }

                _ = doIt;
            }

            for (var i = mesh.Data; i < mesh.NumFaces; i++)
            {
                var tri = mesh.GetFace((int)i);
                var a = mesh.GetPoint(tri.I0);
                var b = mesh.GetPoint(tri.I1);
                var c = mesh.GetPoint(tri.I2);
                result.AddFace(a, b, c, tri.PId);
            }
        }

        public static void doubleClipSingleMesh2(Geometry mesh, BVH bvh1, BVH bvh2, Geometry result)
        {
            for (var i = 0; i < mesh.Planes.Count; i++)
            {
                result.HasPlanes = true;
                result.Planes.Add(mesh.Planes[i]);
            }

            var bvh1Ptr = bvh1.Ptr ?? throw new InvalidOperationException("bvh1.Ptr is null.");
            var bvh2Ptr = bvh2.Ptr ?? throw new InvalidOperationException("bvh2.Ptr is null.");

            for (var i = 0u; i < mesh.Data; i++)
            {
                var tri = mesh.GetFace((int)i);
                var a = mesh.GetPoint(tri.I0);
                var b = mesh.GetPoint(tri.I1);
                var c = mesh.GetPoint(tri.I2);

                var n = computeNormal(a, b, c);
                var area = areaOfTriangle(a, b, c);
                _ = area;

                var triCenter = (a + b * 2.0 + c * 3.0) * (1.0 / 6.0); // Using true centroid could cause issues (#540)
                var raydir = computeNormal(a, b, c);

                var isInside1Loc = IsInsideMesh(triCenter, n, bvh1Ptr, bvh1, raydir, true);
                var isInside2Loc = IsInsideMesh(triCenter, n, bvh2Ptr, bvh2, raydir, true);

                var extraDir1 = Vec3.Normalize(new Vec3(1.1, 1.4, 1.2));
                var extraDir2 = Vec3.Normalize(new Vec3(-2.1, 1.4, -3.2));

                var isInside1LocB = IsInsideMesh(triCenter, n, bvh1Ptr, bvh1, extraDir1, true);
                var isInside2LocB = IsInsideMesh(triCenter, n, bvh2Ptr, bvh2, extraDir1, true);

                if (isInside1Loc.Loc != isInside1LocB.Loc)
                {
                    var isInside1LocC = IsInsideMesh(triCenter, n, bvh1Ptr, bvh1, extraDir2, true);
                    if (isInside1LocC.Loc == isInside1LocB.Loc)
                    {
                        isInside1Loc = isInside1LocB;
                    }
                    else if (isInside1LocB.Loc != isInside1LocC.Loc && isInside1Loc.Loc != isInside1LocC.Loc)
                    {
                        isInside1Loc = isInside1LocB;
                    }
                }

                if (isInside2Loc.Loc != isInside2LocB.Loc)
                {
                    var isInside2LocC = IsInsideMesh(triCenter, n, bvh2Ptr, bvh2, extraDir2, true);
                    if (isInside2LocC.Loc == isInside2LocB.Loc)
                    {
                        isInside2Loc = isInside2LocB;
                    }
                    else if (isInside2LocB.Loc != isInside2LocC.Loc && isInside2Loc.Loc != isInside2LocC.Loc)
                    {
                        isInside2Loc = isInside2LocB;
                    }
                }

                var isInside1 = isInside1Loc.Loc;
                var isInside2 = isInside2Loc.Loc;

                if (isInside1 == MeshLocation.OUTSIDE && isInside2 == MeshLocation.OUTSIDE)
                {
                    // both outside, no dice, should be impossible though
                }
                else if (isInside1 == MeshLocation.INSIDE || isInside2 == MeshLocation.INSIDE)
                {
                    // we only keep boundaries, no dice
                }
                else if (isInside1 == MeshLocation.BOUNDARY && isInside2 == MeshLocation.BOUNDARY)
                {
                    // both boundary, no dice if normals are opposite direction
                    var dot = Vec3.Dot(isInside1Loc.Normal, isInside2Loc.Normal);
                    if (dot > 0)
                    {
                        result.AddFace(a, b, c, tri.PId);
                    }
                }
                else if (isInside1 == MeshLocation.BOUNDARY && isInside2 == MeshLocation.OUTSIDE)
                {
                    if (Vec3.Dot(n, isInside1Loc.Normal) < 0)
                    {
                        result.AddFace(b, a, c, tri.PId);
                    }
                    else
                    {
                        result.AddFace(a, b, c, tri.PId);
                    }
                }
                else if (isInside2 == MeshLocation.BOUNDARY && isInside1 == MeshLocation.OUTSIDE)
                {
                    if (Vec3.Dot(n, isInside2Loc.Normal) < 0)
                    {
                        result.AddFace(b, a, c, tri.PId);
                    }
                    else
                    {
                        result.AddFace(a, b, c, tri.PId);
                    }
                }
                else
                {
                    // neither a boundary, neither inside, neither outside, nothing left
                }
            }

            for (var i = mesh.Data; i < mesh.NumFaces; i++)
            {
                var tri = mesh.GetFace((int)i);
                var a = mesh.GetPoint(tri.I0);
                var b = mesh.GetPoint(tri.I1);
                var c = mesh.GetPoint(tri.I2);
                result.AddFace(a, b, c, tri.PId);
            }
        }

        public static Geometry clipJoin(Geometry mesh, BVH bvh1, BVH bvh2)
        {
            var resultingMesh = new Geometry();
            doubleClipSingleMesh2(mesh, bvh1, bvh2, resultingMesh);
            return resultingMesh;
        }

        public static Geometry clipSubtract(Geometry mesh, BVH bvh1, BVH bvh2)
        {
            var resultingMesh = new Geometry();
            doubleClipSingleMesh(mesh, bvh1, bvh2, resultingMesh);
            return resultingMesh;
        }
    }
}
