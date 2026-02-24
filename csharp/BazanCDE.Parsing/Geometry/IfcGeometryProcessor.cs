using System.Numerics;
using BazanCDE.Parsing.Geometry.Operations;
using BazanCDE.Parsing.Geometry.Operations.BooleanUtils;
using BazanCDE.Parsing.Geometry.Representation;
using BazanCDE.Parsing.Utilities;
using EngineGeometry = BazanCDE.Parsing.Geometry.Operations.BooleanUtils.Geometry;
using EngineSimplePlane = BazanCDE.Parsing.Geometry.Operations.BooleanUtils.SimplePlane;
using BimPlane = BazanCDE.Parsing.Geometry.Operations.BimGeometry.Plane;
using BimEps = BazanCDE.Parsing.Geometry.Operations.BimGeometry.Epsilons;

namespace BazanCDE.Parsing.Geometry;

public sealed class IfcGeometrySettings
{
    public bool _coordinateToOrigin = false;
    public bool _optimize_profiles = true;
    public bool _exportPolylines = false;
    public ushort _circleSegments = 12;
    public double TOLERANCE_PLANE_INTERSECTION = 1.0E-04;
    public double TOLERANCE_PLANE_DEVIATION = 1.0E-04;
    public double TOLERANCE_BACK_DEVIATION_DISTANCE = 1.0E-04;
    public double TOLERANCE_INSIDE_OUTSIDE_PERIMETER = 1.0E-10;
    public double TOLERANCE_BOUNDING_BOX = 1.0E-02;
    public ushort _BOOLEAN_UNION_THRESHOLD = 150;
}

public sealed class BooleanManager
{
    private static double _boolStatus;

    private static EngineGeometry ConvertToEngine(IfcGeometry geom)
    {
        var newGeom = new EngineGeometry
        {
            NumPoints = geom.numPoints,
            NumFaces = geom.numFaces,
            HasPlanes = geom.hasPlanes
        };

        newGeom.FVertexData.AddRange(geom.fvertexData);
        newGeom.VertexData.AddRange(geom.vertexData);
        newGeom.IndexData.AddRange(geom.indexData);
        newGeom.PlaneData.AddRange(geom.planeData);

        foreach (var plane in geom.planes)
        {
            newGeom.Planes.Add(new EngineSimplePlane(plane.normal, plane.distance));
        }

        return newGeom;
    }

    private static IfcGeometry ConvertToWebIfc(EngineGeometry geom)
    {
        var newGeom = new IfcGeometry
        {
            numPoints = geom.NumPoints,
            numFaces = geom.NumFaces,
            hasPlanes = geom.HasPlanes
        };

        newGeom.fvertexData.AddRange(geom.FVertexData);
        newGeom.vertexData.AddRange(geom.VertexData);
        newGeom.indexData.AddRange(geom.IndexData);
        newGeom.planeData.AddRange(geom.PlaneData);

        uint id = 0;
        foreach (var plane in geom.Planes)
        {
            newGeom.planes.Add(new BimPlane
            {
                id = id,
                normal = plane.Normal,
                distance = plane.Distance
            });
            id++;
        }

        return newGeom;
    }

    private static IfcGeometry Union(IfcGeometry firstOperator, IfcGeometry secondOperator)
    {
        var firstEngGeom = ConvertToEngine(firstOperator);
        var secondEngGeom = ConvertToEngine(secondOperator);
        return ConvertToWebIfc(Utils.Union(firstEngGeom, secondEngGeom));
    }

    private static IfcGeometry Subtract(IfcGeometry firstOperator, IfcGeometry secondOperator)
    {
        var firstEngGeom = ConvertToEngine(firstOperator);
        var secondEngGeom = ConvertToEngine(secondOperator);
        return ConvertToWebIfc(Utils.Subtract(firstEngGeom, secondEngGeom));
    }

    public IfcGeometry BoolProcess(IReadOnlyList<IfcGeometry> firstGeoms, List<IfcGeometry> secondGeoms, string op, IfcGeometrySettings settings)
    {
        var finalResult = new IfcGeometry();

        foreach (var firstGeom in firstGeoms)
        {
            var firstOperator = firstGeom;

            foreach (var secondGeom in secondGeoms)
            {
                if (secondGeom.numFaces == 0)
                {
                    continue;
                }

                if (firstOperator.numFaces == 0 && !string.Equals(op, "UNION", StringComparison.Ordinal))
                {
                    break;
                }

                IfcGeometry secondOperator;

                if (secondGeom.halfSpace)
                {
                    var origin = secondGeom.halfSpaceOrigin;
                    var x = secondGeom.halfSpaceX - origin;
                    var y = secondGeom.halfSpaceY - origin;
                    var z = secondGeom.halfSpaceZ - origin;

                    var trans = new double[]
                    {
                        x.X, x.Y, x.Z, 0,
                        y.X, y.Y, y.Z, 0,
                        z.X, z.Y, z.Z, 0,
                        0, 0, 0, 1
                    };

                    var scaleX = 1.0;
                    var scaleY = 1.0;
                    var scaleZ = 1.0;

                    for (uint i = 0; i < firstOperator.numPoints; i++)
                    {
                        var p = firstOperator.GetPoint(i);
                        var vec = p - origin;

                        var dx = Vec3.Dot(vec, x);
                        var dy = Vec3.Dot(vec, y);
                        var dz = Vec3.Dot(vec, z);

                        if (Math.Abs(dx) > scaleX)
                        {
                            scaleX = Math.Abs(dx);
                        }

                        if (Math.Abs(dy) > scaleY)
                        {
                            scaleY = Math.Abs(dy);
                        }

                        if (Math.Abs(dz) > scaleZ)
                        {
                            scaleZ = Math.Abs(dz);
                        }
                    }

                    secondOperator = new IfcGeometry();
                    secondOperator.AddGeometry(secondGeom, trans, scaleX * 2, scaleY * 2, scaleZ * 2, secondGeom.halfSpaceOrigin);
                }
                else
                {
                    secondOperator = secondGeom;
                }

                firstOperator.buildPlanes();
                secondOperator.buildPlanes();

                Utils.SetEpsilons(
                    settings.TOLERANCE_PLANE_INTERSECTION,
                    settings.TOLERANCE_PLANE_DEVIATION,
                    settings.TOLERANCE_BACK_DEVIATION_DISTANCE,
                    settings.TOLERANCE_INSIDE_OUTSIDE_PERIMETER,
                    settings.TOLERANCE_BOUNDING_BOX,
                    _boolStatus);

                if (string.Equals(op, "DIFFERENCE", StringComparison.Ordinal))
                {
                    firstOperator = Subtract(firstOperator, secondOperator);
                }
                else if (string.Equals(op, "UNION", StringComparison.Ordinal))
                {
                    firstOperator = Union(firstOperator, secondOperator);
                }
            }

            finalResult.AddGeometry(firstOperator);
        }

        return finalResult;
    }
}

public sealed class IfcGeometryProcessor
{
    private static readonly double[] Identity4 =
    {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    private readonly Dictionary<string, uint> _typeCodeCache = new(StringComparer.Ordinal);

    private readonly IfcGeometrySettings _settings;
    private readonly Dictionary<uint, IfcGeometry> _expressIDToGeometry;
    private readonly IfcGeometryLoader _geometryLoader;
    private double[] _transformation;
    private readonly IfcLoader _loader;
    private readonly BooleanManager _boolEngine;
    private readonly IIfcSchemaManager _schemaManager;
    private bool _isCoordinated;
    private readonly uint _expressIdCyl;
    private readonly uint _expressIdRect;
    private double[] _coordinationMatrix;
    private readonly IfcGeometry _predefinedCylinder;
    private readonly IfcGeometry _predefinedCube;

    public IfcGeometryProcessor(
        IfcLoader loader,
        IIfcSchemaManager schemaManager,
        ushort circleSegments,
        bool coordinateToOrigin,
        double tolerancePlaneIntersection,
        double tolerancePlaneDeviation,
        double toleranceBackDeviationDistance,
        double toleranceInsideOutsidePerimeter,
        double toleranceScalarEquality,
        double planeRefitIterations,
        double booleanUnionThreshold)
    {
        _loader = loader ?? throw new ArgumentNullException(nameof(loader));
        _schemaManager = schemaManager ?? throw new ArgumentNullException(nameof(schemaManager));

        _geometryLoader = new IfcGeometryLoader(
            loader,
            schemaManager,
            circleSegments,
            tolerancePlaneIntersection,
            tolerancePlaneDeviation,
            toleranceBackDeviationDistance,
            toleranceInsideOutsidePerimeter,
            toleranceScalarEquality,
            planeRefitIterations,
            booleanUnionThreshold);

        _settings = new IfcGeometrySettings
        {
            _coordinateToOrigin = coordinateToOrigin,
            _circleSegments = circleSegments,
            TOLERANCE_PLANE_INTERSECTION = tolerancePlaneIntersection,
            TOLERANCE_PLANE_DEVIATION = tolerancePlaneDeviation,
            TOLERANCE_BACK_DEVIATION_DISTANCE = toleranceBackDeviationDistance,
            TOLERANCE_INSIDE_OUTSIDE_PERIMETER = toleranceInsideOutsidePerimeter,
            _BOOLEAN_UNION_THRESHOLD = (ushort)Math.Clamp((int)booleanUnionThreshold, 0, ushort.MaxValue)
        };

        GeometryUtils.SetEpsilons(toleranceScalarEquality, planeRefitIterations, booleanUnionThreshold);

        _expressIDToGeometry = new Dictionary<uint, IfcGeometry>();
        _transformation = GeometryUtils.FlattenTransformation(Identity4);
        _boolEngine = new BooleanManager();
        _coordinationMatrix = GeometryUtils.FlattenTransformation(Identity4);
        _predefinedCylinder = new IfcGeometry();
        _predefinedCube = new IfcGeometry();
        _isCoordinated = false;
        _expressIdCyl = 0;
        _expressIdRect = 0;
    }

    private IfcGeometryProcessor(
        IfcGeometrySettings settings,
        Dictionary<uint, IfcGeometry> expressIDToGeometry,
        IfcGeometryLoader geometryLoader,
        double[] transformation,
        IfcLoader loader,
        BooleanManager boolEngine,
        IIfcSchemaManager schemaManager,
        bool isCoordinated,
        uint expressIdCyl,
        uint expressIdRect,
        double[] coordinationMatrix,
        IfcGeometry predefinedCylinder,
        IfcGeometry predefinedCube)
    {
        _settings = settings;
        _expressIDToGeometry = expressIDToGeometry;
        _geometryLoader = geometryLoader;
        _transformation = GeometryUtils.FlattenTransformation(transformation);
        _loader = loader;
        _boolEngine = boolEngine;
        _schemaManager = schemaManager;
        _isCoordinated = isCoordinated;
        _expressIdCyl = expressIdCyl;
        _expressIdRect = expressIdRect;
        _coordinationMatrix = GeometryUtils.FlattenTransformation(coordinationMatrix);
        _predefinedCylinder = predefinedCylinder;
        _predefinedCube = predefinedCube;
    }

    public IfcGeometryLoader GetLoader()
    {
        return _geometryLoader;
    }

    public void SetTransformation(double[] val)
    {
        if (val is null || val.Length < 16)
        {
            throw new ArgumentException("Transformation matrix must have 16 elements.", nameof(val));
        }

        _transformation = new double[16];
        Array.Copy(val, _transformation, 16);
    }

    public IfcGeometry GetGeometry(uint expressID)
    {
        if (!_expressIDToGeometry.TryGetValue(expressID, out var geometry))
        {
            geometry = new IfcGeometry();
            _expressIDToGeometry[expressID] = geometry;
        }

        return geometry;
    }

    public void Clear()
    {
        _expressIDToGeometry.Clear();
        _geometryLoader.Clear();
    }

    public double[] GetFlatCoordinationMatrix()
    {
        return GeometryUtils.FlattenTransformation(_coordinationMatrix);
    }

    public double[] GetCoordinationMatrix()
    {
        return GeometryUtils.FlattenTransformation(_coordinationMatrix);
    }

    private Vec4? GetStyleItemFromExpressId(uint expressID)
    {
        Vec4? styledItemColor = null;

        var styledItems = _geometryLoader.GetStyledItems();
        var relMaterials = _geometryLoader.GetRelMaterials();
        var materialDefinitions = _geometryLoader.GetMaterialDefinitions();

        if (styledItems.TryGetValue(expressID, out var items))
        {
            foreach (var item in items)
            {
                styledItemColor = _geometryLoader.GetColor(item.StyleId);
                if (styledItemColor.HasValue)
                {
                    break;
                }
            }
        }

        if (!styledItemColor.HasValue && relMaterials.TryGetValue(expressID, out var materials))
        {
            foreach (var item in materials)
            {
                if (materialDefinitions.TryGetValue(item.MaterialId, out var defs))
                {
                    foreach (var def in defs)
                    {
                        styledItemColor = _geometryLoader.GetColor(def.DefinitionId);
                        if (styledItemColor.HasValue)
                        {
                            break;
                        }
                    }
                }

                if (!styledItemColor.HasValue)
                {
                    styledItemColor = _geometryLoader.GetColor(item.MaterialId);
                }

                if (styledItemColor.HasValue)
                {
                    break;
                }
            }
        }

        return styledItemColor;
    }

    public IfcComposedMesh GetMesh(uint expressID)
    {
        var lineType = _loader.GetLineType(expressID);
        var relVoids = _geometryLoader.GetRelVoids();

        var mesh = new IfcComposedMesh
        {
            expressID = expressID,
            transformation = GeometryUtils.FlattenTransformation(Identity4)
        };

        var generatedColor = GetStyleItemFromExpressId(expressID);
        if (!generatedColor.HasValue)
        {
            mesh.color = new Vec4(1, 1, 1, 1);
            mesh.hasColor = false;
        }
        else
        {
            mesh.color = generatedColor.Value;
            mesh.hasColor = true;
        }

        if (_schemaManager.IsIfcElement(lineType))
        {
            _loader.MoveToArgumentOffset(expressID, 5);

            uint localPlacement = 0;
            if (_loader.GetTokenType() == IfcTokenType.Ref)
            {
                _loader.StepBack();
                localPlacement = _loader.GetRefArgument();
            }

            uint ifcPresentation = 0;
            if (_loader.GetTokenType() == IfcTokenType.Ref)
            {
                _loader.StepBack();
                ifcPresentation = _loader.GetRefArgument();
            }

            if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
            {
                mesh.transformation = _geometryLoader.GetLocalPlacement(localPlacement);
            }

            if (ifcPresentation != 0 && _loader.IsValidExpressID(ifcPresentation))
            {
                mesh.children.Add(GetMesh(ifcPresentation));
            }

            if (relVoids.TryGetValue(expressID, out var relVoidIds) && relVoidIds.Count > 0)
            {
                var origin = GeometryUtils.GetOrigin(mesh, _expressIDToGeometry);
                var normalizeMat = TranslationMatrix(origin * -1.0);

                var voidGeoms = new List<IfcGeometry>();
                foreach (var relVoidExpressID in relVoidIds)
                {
                    var voidGeom = GetMesh(relVoidExpressID);
                    var flatVoidMesh = GeometryUtils.flatten(voidGeom, _expressIDToGeometry, normalizeMat);
                    voidGeoms.AddRange(flatVoidMesh);
                }

                if (relVoidIds.Count > _settings._BOOLEAN_UNION_THRESHOLD)
                {
                    var joinedVoidGeoms = new List<IfcGeometry>();
                    var fusedVoids = new IfcGeometry();

                    foreach (var geom in voidGeoms)
                    {
                        if (geom.halfSpace)
                        {
                            joinedVoidGeoms.Add(geom);
                        }
                        else
                        {
                            fusedVoids = BoolProcess([fusedVoids], new List<IfcGeometry> { geom }, "UNION", _settings);
                        }
                    }

                    joinedVoidGeoms.Add(fusedVoids);
                    voidGeoms = joinedVoidGeoms;
                }

                ApplyBooleanToMeshChildren(mesh, voidGeoms, "DIFFERENCE", _settings, normalizeMat);
                return mesh;
            }

            return mesh;
        }

        if (lineType == Type("IFCSECTIONEDSOLIDHORIZONTAL") ||
            lineType == Type("IFCSECTIONEDSOLID") ||
            lineType == Type("IFCSECTIONEDSURFACE"))
        {
            var geom = GeometryUtils.SectionedSurface(_geometryLoader.GetCrossSections3D(expressID), true);
            mesh.transformation = GeometryUtils.FlattenTransformation(Identity4);
            _expressIDToGeometry[expressID] = geom;
            mesh.hasGeometry = true;
            return mesh;
        }

        if (lineType == Type("IFCMAPPEDITEM"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var ifcPresentation = _loader.GetRefArgument();
            var localPlacement = _loader.GetRefArgument();

            mesh.transformation = _geometryLoader.GetLocalPlacement(localPlacement);
            mesh.children.Add(GetMesh(ifcPresentation));

            return mesh;
        }

        if (lineType == Type("IFCBOOLEANCLIPPINGRESULT"))
        {
            _loader.MoveToArgumentOffset(expressID, 1);
            var firstOperandID = _loader.GetRefArgument();
            var secondOperandID = _loader.GetRefArgument();

            var firstMesh = GetMesh(firstOperandID);
            var secondMesh = GetMesh(secondOperandID);

            var origin = GeometryUtils.GetOrigin(firstMesh, _expressIDToGeometry);
            var normalizeMat = TranslationMatrix(origin * -1.0);

            var flatFirstMeshes = GeometryUtils.flatten(firstMesh, _expressIDToGeometry, normalizeMat);
            var flatSecondMeshes = GeometryUtils.flatten(secondMesh, _expressIDToGeometry, normalizeMat);

            var resultMesh = BoolProcess(flatFirstMeshes, flatSecondMeshes, "DIFFERENCE", _settings);

            _expressIDToGeometry[expressID] = resultMesh;
            mesh.hasGeometry = true;
            mesh.transformation = TranslationMatrix(origin);

            if (!mesh.hasColor && firstMesh.hasColor)
            {
                mesh.hasColor = true;
                mesh.color = firstMesh.color;
            }

            return mesh;
        }

        if (lineType == Type("IFCBOOLEANRESULT"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var op = _loader.GetStringArgument();

            if (!string.Equals(op, "DIFFERENCE", StringComparison.Ordinal) &&
                !string.Equals(op, "UNION", StringComparison.Ordinal))
            {
                return mesh;
            }

            var firstOperandID = _loader.GetRefArgument();
            var secondOperandID = _loader.GetRefArgument();

            var firstMesh = GetMesh(firstOperandID);
            var secondMesh = GetMesh(secondOperandID);

            var origin = GeometryUtils.GetOrigin(firstMesh, _expressIDToGeometry);
            var normalizeMat = TranslationMatrix(origin * -1.0);

            var flatFirstMeshes = GeometryUtils.flatten(firstMesh, _expressIDToGeometry, normalizeMat);
            var flatSecondMeshes = GeometryUtils.flatten(secondMesh, _expressIDToGeometry, normalizeMat);

            if (flatFirstMeshes.Count == 0)
            {
                return mesh;
            }

            var resultMesh = BoolProcess(flatFirstMeshes, flatSecondMeshes, op, _settings);

            _expressIDToGeometry[expressID] = resultMesh;
            mesh.hasGeometry = true;
            mesh.transformation = TranslationMatrix(origin);

            if (!mesh.hasColor && firstMesh.hasColor)
            {
                mesh.hasColor = true;
                mesh.color = firstMesh.color;
            }

            return mesh;
        }

        if (lineType == Type("IFCHALFSPACESOLID"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var surfaceID = _loader.GetRefArgument();
            var agreement = _loader.GetStringArgument();

            var surface = GetSurface(surfaceID);

            var extrusionNormal = new Vec3(0, 0, 1);
            var flipWinding = false;
            if (string.Equals(agreement, "T", StringComparison.Ordinal))
            {
                extrusionNormal *= -1;
                flipWinding = true;
            }

            var d = IfcRepresentationConstants.EXTRUSION_DISTANCE_HALFSPACE_M / _geometryLoader.GetLinearScalingFactor();

            var profile = new IfcProfile
            {
                isConvex = false,
                curve = CurveUtils.GetRectangleCurve(d, d, null)
            };

            var geom = GeometryUtils.Extrude(profile, extrusionNormal, d);
            geom.halfSpace = true;

            if (flipWinding)
            {
                FlipFaceWinding(geom);
            }

            mesh.transformation = surface.transformation;
            _expressIDToGeometry[expressID] = geom;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCPOLYGONALBOUNDEDHALFSPACE"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var surfaceID = _loader.GetRefArgument();
            var agreement = _loader.GetStringArgument();
            var positionID = _loader.GetRefArgument();
            var boundaryID = _loader.GetRefArgument();

            var surface = GetSurface(surfaceID);
            var position = _geometryLoader.GetLocalPlacement(positionID);
            var curve = _geometryLoader.GetCurve(boundaryID, 2);

            if (!curve.IsCCW())
            {
                curve.Invert();
            }

            var extrusionNormal = new Vec3(0, 0, 1);
            var planeNormal = GetMatrixColumn(surface.transformation, 2);
            var planePosition = GetMatrixTranslation(surface.transformation);

            var invPosition = InvertMatrix4(position);
            var localPlaneNormal = TransformDirection(invPosition, planeNormal);
            var localPlanePos = TransformPoint(invPosition, planePosition);

            var flipWinding = false;
            var extrudeDistance = IfcRepresentationConstants.EXTRUSION_DISTANCE_HALFSPACE_M / _geometryLoader.GetLinearScalingFactor();

            var halfSpaceInPlaneDirection = !string.Equals(agreement, "T", StringComparison.Ordinal);
            var extrudeInPlaneDirection = Vec3.Dot(localPlaneNormal, extrusionNormal) > 0;
            var ignoreDistanceInExtrude = (!halfSpaceInPlaneDirection && extrudeInPlaneDirection)
                                          || (halfSpaceInPlaneDirection && !extrudeInPlaneDirection);
            if (ignoreDistanceInExtrude)
            {
                extrudeDistance *= -1;
                flipWinding = true;
            }

            var profile = new IfcProfile
            {
                isConvex = false,
                curve = curve
            };

            var geom = GeometryUtils.Extrude(profile, extrusionNormal, extrudeDistance, localPlaneNormal, localPlanePos);

            if (flipWinding)
            {
                FlipFaceWinding(geom);
            }

            _expressIDToGeometry[expressID] = geom;
            mesh.hasGeometry = true;
            mesh.transformation = position;

            return mesh;
        }

        if (lineType == Type("IFCREPRESENTATIONMAP"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var axis2Placement = _loader.GetRefArgument();
            var ifcPresentation = _loader.GetRefArgument();

            mesh.transformation = _geometryLoader.GetLocalPlacement(axis2Placement);
            mesh.children.Add(GetMesh(ifcPresentation));

            return mesh;
        }

        if (lineType == Type("IFCFACEBASEDSURFACEMODEL") || lineType == Type("IFCSHELLBASEDSURFACEMODEL"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var shells = _loader.GetSetArgument();

            foreach (var shell in shells)
            {
                var shellRef = _loader.GetRefArgument(shell);
                var temp = new IfcComposedMesh
                {
                    expressID = shellRef,
                    hasGeometry = true,
                    transformation = GeometryUtils.FlattenTransformation(Identity4)
                };

                _expressIDToGeometry[shellRef] = GetBrep(shellRef);

                var shellColor = GetStyleItemFromExpressId(shellRef);
                if (shellColor.HasValue)
                {
                    temp.color = shellColor.Value;
                    temp.hasColor = true;
                }

                mesh.children.Add(temp);
            }

            var unitaryFaces = 0;
            foreach (var child in mesh.children)
            {
                if (_expressIDToGeometry.TryGetValue(child.expressID, out var temp) && temp.numFaces < 4)
                {
                    unitaryFaces++;
                }
            }

            if (unitaryFaces > 12)
            {
                var newGeometry = new IfcGeometry();
                foreach (var child in mesh.children)
                {
                    if (_expressIDToGeometry.TryGetValue(child.expressID, out var temp))
                    {
                        newGeometry.AddGeometry(temp);
                    }
                }

                _expressIDToGeometry[expressID] = newGeometry;

                var newMesh = new IfcComposedMesh
                {
                    expressID = expressID,
                    hasGeometry = true,
                    transformation = GeometryUtils.FlattenTransformation(Identity4)
                };

                var shellColor = GetStyleItemFromExpressId(expressID);
                if (shellColor.HasValue)
                {
                    newMesh.color = shellColor.Value;
                    newMesh.hasColor = true;
                }

                return newMesh;
            }

            return mesh;
        }

        if (lineType == Type("IFCADVANCEDBREP") || lineType == Type("IFCFACETEDBREP"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var ifcPresentation = _loader.GetRefArgument();

            _expressIDToGeometry[expressID] = GetBrep(ifcPresentation);
            if (!mesh.hasColor)
            {
                mesh.color = GetStyleItemFromExpressId(ifcPresentation) ?? new Vec4(1, 1, 1, 1);
            }

            mesh.hasGeometry = true;
            return mesh;
        }

        if (lineType == Type("IFCPRODUCTREPRESENTATION") || lineType == Type("IFCPRODUCTDEFINITIONSHAPE"))
        {
            _loader.MoveToArgumentOffset(expressID, 2);
            var representations = _loader.GetSetArgument();

            foreach (var repToken in representations)
            {
                var repID = _loader.GetRefArgument(repToken);
                mesh.children.Add(GetMesh(repID));
            }

            return mesh;
        }

        if (lineType == Type("IFCTOPOLOGYREPRESENTATION") || lineType == Type("IFCSHAPEREPRESENTATION"))
        {
            _loader.MoveToArgumentOffset(expressID, 1);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 3);
            var repItems = _loader.GetSetArgument();

            foreach (var repToken in repItems)
            {
                var repID = _loader.GetRefArgument(repToken);
                mesh.children.Add(GetMesh(repID));
            }

            return mesh;
        }

        if (lineType == Type("IFCPOLYGONALFACESET"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var coordinatesRef = _loader.GetRefArgument();
            var points = _geometryLoader.ReadIfcCartesianPointList3D(coordinatesRef);

            _loader.MoveToArgumentOffset(expressID, 2);
            var faces = _loader.GetSetArgument();

            var geom = new IfcGeometry();
            var bounds = new List<IfcBound3D>();

            foreach (var face in faces)
            {
                var faceID = _loader.GetRefArgument(face);
                ReadIndexedPolygonalFace(faceID, bounds, points);
                GeometryUtils.TriangulateBounds(geom, bounds, expressID);
                bounds.Clear();
            }

            _loader.MoveToArgumentOffset(expressID, 3);
            if (_loader.GetTokenType() == IfcTokenType.SetBegin)
            {
                // Unsupported IFCPOLYGONALFACESET with PnIndex.
            }

            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCFACESURFACE"))
        {
            var geometry = new IfcGeometry();

            _loader.MoveToArgumentOffset(expressID, 0);
            var bounds = _loader.GetSetArgument();

            var bounds3D = new List<IfcBound3D>(bounds.Count);
            foreach (var bound in bounds)
            {
                var boundID = _loader.GetRefArgument(bound);
                bounds3D.Add(_geometryLoader.GetBound(boundID));
            }

            GeometryUtils.TriangulateBounds(geometry, bounds3D, expressID);

            _loader.MoveToArgumentOffset(expressID, 1);
            var surfRef = _loader.GetRefArgument();

            var surface = GetSurface(surfRef);

            if (surface.BSplineSurface.Active)
            {
                TriangulateBspline(geometry, bounds3D, surface, _geometryLoader.GetLinearScalingFactor());
            }
            else if (surface.CylinderSurface.Active)
            {
                TriangulateCylindricalSurface(geometry, bounds3D, surface, _settings._circleSegments);
            }
            else if (surface.RevolutionSurface.Active)
            {
                TriangulateRevolution(geometry, bounds3D, surface, _settings._circleSegments);
            }
            else if (surface.ExtrusionSurface.Active)
            {
                TriangulateExtrusion(geometry, bounds3D, surface);
            }
            else
            {
                GeometryUtils.TriangulateBounds(geometry, bounds3D, expressID);
            }

            _expressIDToGeometry[expressID] = geometry;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCTRIANGULATEDIRREGULARNETWORK") || lineType == Type("IFCTRIANGULATEDFACESET"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var coordinatesRef = _loader.GetRefArgument();
            var points = _geometryLoader.ReadIfcCartesianPointList3D(coordinatesRef);

            _loader.MoveToArgumentOffset(expressID, 3);
            var indices = Read2DArrayOfThreeIndices();

            var geom = new IfcGeometry();

            _loader.MoveToArgumentOffset(expressID, 4);
            if (_loader.GetTokenType() == IfcTokenType.SetBegin)
            {
                _loader.StepBack();
                _ = Read2DArrayOfThreeIndices();
            }

            for (var i = 0; i + 2 < indices.Count; i += 3)
            {
                var i1 = (int)indices[i + 0] - 1;
                var i2 = (int)indices[i + 1] - 1;
                var i3 = (int)indices[i + 2] - 1;

                if (i1 >= 0 && i2 >= 0 && i3 >= 0 && i1 < points.Count && i2 < points.Count && i3 < points.Count)
                {
                    geom.AddFace(points[i1], points[i2], points[i3]);
                }
            }

            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCSURFACECURVESWEPTAREASOLID"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);

            var profileID = _loader.GetRefArgument();
            var placementID = _loader.GetRefArgument();
            var directrixRef = _loader.GetRefArgument();

            var startParam = 0.0;
            var endParam = 1.0;
            var closed = false;

            if (_loader.GetTokenType() == IfcTokenType.Real)
            {
                _loader.StepBack();
                startParam = _loader.GetDoubleArgument();
            }

            if (_loader.GetTokenType() == IfcTokenType.Real)
            {
                _loader.StepBack();
                endParam = _loader.GetDoubleArgument();
            }

            var surfaceID = _loader.GetRefArgument();

            if (profileID == 0 || directrixRef == 0 || surfaceID == 0)
            {
                return mesh;
            }

            var profile = _geometryLoader.GetProfile(profileID);
            var placement = placementID != 0 ? _geometryLoader.GetLocalPlacement(placementID) : GeometryUtils.FlattenTransformation(Identity4);
            var directrix = _geometryLoader.GetCurve(directrixRef, 3);
            var surface = GetSurface(surfaceID);

            if (directrix.points.Count > 1)
            {
                var dst = (directrix.points[0] - directrix.points[^1]).Length();
                if (Math.Abs(startParam) < 1e-9 && Math.Abs(endParam - 1) < 1e-9 && dst < 1e-5)
                {
                    closed = true;
                }
            }

            profile.curve.points.Reverse();

            var geom = GeometryUtils.Sweep(
                _geometryLoader.GetLinearScalingFactor(),
                closed,
                profile,
                directrix,
                surface.normal(),
                true);

            mesh.transformation = placement;
            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCFIXEDREFERENCESWEPTAREASOLID"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var profileID = _loader.GetRefArgument();
            var placementID = _loader.GetOptionalRefArgument();
            var directrixRef = _loader.GetRefArgument();
            var fixedReferenceID = _loader.GetRefArgument();

            var profile = _geometryLoader.GetProfile(profileID);
            var placement = placementID != 0 ? _geometryLoader.GetLocalPlacement(placementID) : GeometryUtils.FlattenTransformation(Identity4);
            var directrix = _geometryLoader.GetCurve(directrixRef, 3);
            var fixedReference = _geometryLoader.GetCartesianPoint3D(fixedReferenceID);

            if (profile.curve.points.Count == 0 || directrix.points.Count == 0)
            {
                return mesh;
            }

            var closed = (directrix.points[0] - directrix.points[^1]).Length() < IfcRepresentationConstants.EPS_SMALL;

            var geom = GeometryUtils.SweepFixedReference(
                _geometryLoader.GetLinearScalingFactor(),
                closed,
                profile,
                directrix,
                fixedReference);

            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;
            mesh.transformation = placement;

            return mesh;
        }

        if (lineType == Type("IFCSWEPTDISKSOLID"))
        {
            var closed = false;

            _loader.MoveToArgumentOffset(expressID, 0);
            var directrixRef = _loader.GetRefArgument();

            var radius = _loader.GetDoubleArgument();

            if (_loader.GetTokenType() == IfcTokenType.Real)
            {
                _loader.StepBack();
                _ = _loader.GetDoubleArgument();
            }

            if (_loader.GetTokenType() == IfcTokenType.Real)
            {
                _loader.StepBack();
                _ = _loader.GetDoubleArgument();
            }

            if (_loader.GetTokenType() == IfcTokenType.Real)
            {
                _loader.StepBack();
                _ = _loader.GetDoubleArgument();
            }

            var directrix = _geometryLoader.GetCurve(directrixRef, 3);

            var profile = new IfcProfile
            {
                curve = CurveUtils.GetCircleCurve((float)radius, _settings._circleSegments)
            };

            var geom = GeometryUtils.SweepCircular(
                _geometryLoader.GetLinearScalingFactor(),
                closed,
                profile,
                radius,
                directrix);

            geom.sweptDiskSolid.axis = new List<IfcCurve> { directrix };
            geom.sweptDiskSolid.profiles = new List<IfcProfile> { profile };
            geom.sweptDiskSolid.profileRadius = radius;

            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCREVOLVEDAREASOLID"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var profileID = _loader.GetRefArgument();
            var placementID = _loader.GetRefArgument();
            var axis1PlacementID = _loader.GetRefArgument();
            var angle = GeometryUtils.angleConversion(_loader.GetDoubleArgument(), _geometryLoader.GetAngleUnits());

            var profile = _geometryLoader.GetProfile(profileID);
            var placement = _geometryLoader.GetLocalPlacement(placementID);
            var axisPlacement = _geometryLoader.GetAxis1Placement(axis1PlacementID);
            var axis = axisPlacement[0];
            var pos = axisPlacement[1];

            var directrix = CurveUtils.BuildArc(
                _geometryLoader.GetLinearScalingFactor(),
                pos,
                axis,
                angle,
                _settings._circleSegments);

            var closed = directrix.points.Count > 1 &&
                         (directrix.points[0] - directrix.points[^1]).Length() < BimEps.EPS_BIG;

            var geom = new IfcGeometry();

            if (!profile.isComposite)
            {
                geom = GeometryUtils.Sweep(
                    _geometryLoader.GetLinearScalingFactor(),
                    closed,
                    profile,
                    directrix,
                    axis,
                    false);
            }
            else
            {
                for (var i = 0; i < profile.profiles.Count; i++)
                {
                    var geomTemp = GeometryUtils.Sweep(
                        _geometryLoader.GetLinearScalingFactor(),
                        closed,
                        profile.profiles[i],
                        directrix,
                        axis,
                        false,
                        false);

                    geom.AddPart(geomTemp);
                    geom.AddGeometry(geomTemp);
                }
            }

            mesh.transformation = placement;
            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;
            mesh.color = generatedColor ?? new Vec4(1, 1, 1, 1);

            return mesh;
        }

        if (lineType == Type("IFCEXTRUDEDAREASOLID") || lineType == Type("IFCEXTRUDEDAREASOLIDTAPERED"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var profileID = _loader.GetRefArgument();
            var placementID = _loader.GetOptionalRefArgument();
            var directionID = _loader.GetRefArgument();
            var depth = _loader.GetDoubleArgument();

            var profile = _geometryLoader.GetProfile(profileID);
            if (!profile.isComposite)
            {
                if (profile.curve.points.Count == 0)
                {
                    return mesh;
                }
            }
            else
            {
                foreach (var subProfile in profile.profiles)
                {
                    if (subProfile.curve.points.Count == 0)
                    {
                        return mesh;
                    }
                }
            }

            if (placementID != 0)
            {
                mesh.transformation = _geometryLoader.GetLocalPlacement(placementID);
            }

            var dir = _geometryLoader.GetCartesianPoint3D(directionID);
            var dirDot = Vec3.Dot(dir, new Vec3(0, 0, 1));
            var flipWinding = dirDot < 0;

            var geom = new IfcGeometry();

            if (!profile.isComposite)
            {
                geom = GeometryUtils.Extrude(profile, dir, depth);
                if (flipWinding)
                {
                    FlipFaceWinding(geom);
                }
            }
            else
            {
                foreach (var subProfile in profile.profiles)
                {
                    var geomTemp = GeometryUtils.Extrude(subProfile, dir, depth);
                    if (flipWinding)
                    {
                        FlipFaceWinding(geomTemp);
                    }

                    geom.AddPart(geomTemp);
                    geom.AddGeometry(geomTemp);
                }
            }

            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCRIGHTCIRCULARCYLINDER"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var placementID = _loader.GetRefArgument();
            var height = _loader.GetDoubleArgument();
            var radius = _loader.GetDoubleArgument();

            var profile = new IfcProfile
            {
                isConvex = true,
                curve = CurveUtils.GetCircleCurve((float)radius, _settings._circleSegments)
            };

            var geom = GeometryUtils.Extrude(profile, new Vec3(0, 0, 1), height);

            if (placementID != 0)
            {
                mesh.transformation = _geometryLoader.GetLocalPlacement(placementID);
            }

            _expressIDToGeometry[expressID] = geom;
            mesh.expressID = expressID;
            mesh.hasGeometry = true;

            return mesh;
        }

        if (lineType == Type("IFCGEOMETRICSET") || lineType == Type("IFCGEOMETRICCURVESET"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var items = _loader.GetSetArgument();

            foreach (var item in items)
            {
                var itemID = _loader.GetRefArgument(item);
                mesh.children.Add(GetMesh(itemID));
            }

            return mesh;
        }

        if (lineType == Type("IFCBOUNDINGBOX"))
        {
            return mesh;
        }

        if (lineType == Type("IFCCARTESIANPOINT"))
        {
            var geom = new IfcGeometry();
            var point = _geometryLoader.GetCartesianPoint3D(expressID);

            geom.vertexData.Add(point.X);
            geom.vertexData.Add(point.Y);
            geom.vertexData.Add(point.Z);
            geom.vertexData.Add(0);
            geom.vertexData.Add(0);
            geom.vertexData.Add(1);
            geom.indexData.Add(0);

            geom.numPoints = 1;
            geom.isPolygon = true;

            mesh.hasGeometry = true;
            _expressIDToGeometry[expressID] = geom;

            return mesh;
        }

        if (lineType == Type("IFCEDGE") || lineType == Type("IFCEDGECURVE"))
        {
            var edge = _geometryLoader.GetEdge(expressID);
            var geom = new IfcGeometry();

            for (uint i = 0; i < edge.points.Count; i++)
            {
                var vert = edge.points[(int)i];
                geom.vertexData.Add(vert.X);
                geom.vertexData.Add(vert.Y);
                geom.vertexData.Add(vert.Z);
                geom.vertexData.Add(0);
                geom.vertexData.Add(0);
                geom.vertexData.Add(1);
                geom.indexData.Add(i);
            }

            geom.numPoints = (uint)edge.points.Count;
            geom.isPolygon = true;
            mesh.hasGeometry = true;
            _expressIDToGeometry[expressID] = geom;

            return mesh;
        }

        if (lineType == Type("IFCSPHERE"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var placementID = _loader.GetRefArgument();
            var radius = _loader.GetDoubleArgument();

            var placement = _geometryLoader.GetLocalPlacement(placementID);

            var center = GetMatrixTranslation(placement);
            var xAxis = Vec3.Normalize(GetMatrixColumn(placement, 0));
            var yAxis = Vec3.Normalize(GetMatrixColumn(placement, 1));
            var zAxis = Vec3.Normalize(GetMatrixColumn(placement, 2));

            uint azimuthSegments = _settings._circleSegments;
            uint altitudeSegments = Math.Max((uint)(_settings._circleSegments / 2), 4u);

            var geom = new IfcGeometry();
            var vertices = new List<Vec3>((int)((altitudeSegments + 1) * (azimuthSegments + 1)));

            for (uint lat = 0; lat <= altitudeSegments; ++lat)
            {
                var theta = Math.PI * lat / altitudeSegments;
                var sinTheta = Math.Sin(theta);
                var cosTheta = Math.Cos(theta);

                for (uint lon = 0; lon <= azimuthSegments; ++lon)
                {
                    var phi = 2.0 * Math.PI * lon / azimuthSegments;
                    var sinPhi = Math.Sin(phi);
                    var cosPhi = Math.Cos(phi);

                    var localPos = new Vec3(
                        radius * sinTheta * cosPhi,
                        radius * sinTheta * sinPhi,
                        radius * cosTheta);

                    var pos = center + localPos.X * xAxis + localPos.Y * yAxis + localPos.Z * zAxis;
                    vertices.Add(pos);
                }
            }

            foreach (var v in vertices)
            {
                geom.vertexData.Add(v.X);
                geom.vertexData.Add(v.Y);
                geom.vertexData.Add(v.Z);
                geom.vertexData.Add(0.0);
                geom.vertexData.Add(0.0);
                geom.vertexData.Add(1.0);
            }

            var vertsPerRing = azimuthSegments + 1;
            for (uint lat = 0; lat < altitudeSegments; ++lat)
            {
                var bottom = lat * vertsPerRing;
                var top = bottom + vertsPerRing;

                for (uint lon = 0; lon < azimuthSegments; ++lon)
                {
                    var bl = bottom + lon;
                    var br = bottom + lon + 1;
                    var tl = top + lon;
                    var tr = top + lon + 1;

                    geom.indexData.Add(bl);
                    geom.indexData.Add(br);
                    geom.indexData.Add(tl);

                    geom.indexData.Add(br);
                    geom.indexData.Add(tr);
                    geom.indexData.Add(tl);
                }
            }

            geom.numFaces = (uint)(geom.indexData.Count / 3);
            geom.numPoints = (uint)vertices.Count;
            geom.isPolygon = false;
            geom.buildPlanes();

            _expressIDToGeometry[expressID] = geom;
            mesh.hasGeometry = true;
            mesh.expressID = expressID;
            mesh.transformation = placement;

            return mesh;
        }

        if (lineType == Type("IFCCIRCLE") ||
            lineType == Type("IFCCOMPOSITECURVE") ||
            lineType == Type("IFCPOLYLINE") ||
            lineType == Type("IFCINDEXEDPOLYCURVE") ||
            lineType == Type("IFCTRIMMEDCURVE") ||
            lineType == Type("IFCGRADIENTCURVE"))
        {
            var curve = _geometryLoader.GetCurve(expressID, 3, false);

            if (curve.points.Count > 0)
            {
                var geom = new IfcGeometry();

                for (uint i = 0; i < curve.points.Count; i++)
                {
                    var vert = curve.points[(int)i];
                    geom.vertexData.Add(vert.X);
                    geom.vertexData.Add(vert.Y);
                    geom.vertexData.Add(vert.Z);
                    geom.vertexData.Add(0);
                    geom.vertexData.Add(0);
                    geom.vertexData.Add(1);
                    geom.indexData.Add(i);
                }

                geom.numPoints = (uint)curve.points.Count;
                geom.isPolygon = true;
                mesh.hasGeometry = true;
                _expressIDToGeometry[expressID] = geom;
            }

            return mesh;
        }

        if (lineType == Type("IFCTEXTLITERAL") || lineType == Type("IFCTEXTLITERALWITHEXTENT"))
        {
            return mesh;
        }

        return new IfcComposedMesh();
    }

    public IfcSurface GetSurface(uint expressID)
    {
        var lineType = _loader.GetLineType(expressID);

        if (lineType == Type("IFCPLANE"))
        {
            var surface = new IfcSurface();

            _loader.MoveToArgumentOffset(expressID, 0);
            var locationID = _loader.GetRefArgument();
            surface.transformation = _geometryLoader.GetLocalPlacement(locationID);

            return surface;
        }

        if (lineType == Type("IFCBSPLINESURFACE"))
        {
            var surface = new IfcSurface();

            var controlPts = new List<List<Vec3>>();

            _loader.MoveToArgumentOffset(expressID, 0);
            var uDegree = _loader.GetIntArgument();

            _loader.MoveToArgumentOffset(expressID, 1);
            var vDegree = _loader.GetIntArgument();

            _loader.MoveToArgumentOffset(expressID, 2);
            var ctrlPointGroups = _loader.GetSetListArgument();
            foreach (var set in ctrlPointGroups)
            {
                var list = new List<Vec3>();
                foreach (var token in set)
                {
                    var pointID = _loader.GetRefArgument(token);
                    list.Add(_geometryLoader.GetCartesianPoint3D(pointID));
                }

                controlPts.Add(list);
            }

            _loader.MoveToArgumentOffset(expressID, 3);
            var curveType = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 4);
            var closedU = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 5);
            var closedV = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 6);
            _ = _loader.GetStringArgument();

            surface.BSplineSurface.Active = true;
            surface.BSplineSurface.UDegree = uDegree;
            surface.BSplineSurface.VDegree = vDegree;
            surface.BSplineSurface.ControlPoints = controlPts;
            surface.BSplineSurface.ClosedU = closedU;
            surface.BSplineSurface.ClosedV = closedV;
            surface.BSplineSurface.CurveType = curveType;

            return surface;
        }

        if (lineType == Type("IFCBSPLINESURFACEWITHKNOTS"))
        {
            var surface = new IfcSurface();

            var controlPts = new List<List<Vec3>>();
            var uMultiplicity = new List<uint>();
            var vMultiplicity = new List<uint>();
            var uKnots = new List<double>();
            var vKnots = new List<double>();

            _loader.MoveToArgumentOffset(expressID, 0);
            var uDegree = _loader.GetIntArgument();

            _loader.MoveToArgumentOffset(expressID, 1);
            var vDegree = _loader.GetIntArgument();

            _loader.MoveToArgumentOffset(expressID, 2);
            var ctrlPointGroups = _loader.GetSetListArgument();
            foreach (var set in ctrlPointGroups)
            {
                var list = new List<Vec3>();
                foreach (var token in set)
                {
                    var pointID = _loader.GetRefArgument(token);
                    list.Add(_geometryLoader.GetCartesianPoint3D(pointID));
                }

                controlPts.Add(list);
            }

            _loader.MoveToArgumentOffset(expressID, 3);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 4);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 5);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 6);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 7);
            var knotSetU = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(expressID, 8);
            var knotSetV = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(expressID, 9);
            var indexesSetU = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(expressID, 10);
            var indexesSetV = _loader.GetSetArgument();

            foreach (var token in knotSetU)
            {
                uMultiplicity.Add((uint)_loader.GetIntArgument(token));
            }

            foreach (var token in knotSetV)
            {
                vMultiplicity.Add((uint)_loader.GetIntArgument(token));
            }

            foreach (var token in indexesSetU)
            {
                uKnots.Add(_loader.GetDoubleArgument(token));
            }

            foreach (var token in indexesSetV)
            {
                vKnots.Add(_loader.GetDoubleArgument(token));
            }

            surface.BSplineSurface.Active = true;
            surface.BSplineSurface.UDegree = uDegree;
            surface.BSplineSurface.VDegree = vDegree;
            surface.BSplineSurface.ControlPoints = controlPts;
            surface.BSplineSurface.UMultiplicity = uMultiplicity;
            surface.BSplineSurface.VMultiplicity = vMultiplicity;
            surface.BSplineSurface.UKnots = uKnots;
            surface.BSplineSurface.VKnots = vKnots;

            return surface;
        }

        if (lineType == Type("IFCRATIONALBSPLINESURFACEWITHKNOTS"))
        {
            var surface = new IfcSurface();

            var controlPts = new List<List<Vec3>>();
            var weightPts = new List<List<double>>();
            var uMultiplicity = new List<uint>();
            var vMultiplicity = new List<uint>();
            var uKnots = new List<double>();
            var vKnots = new List<double>();

            _loader.MoveToArgumentOffset(expressID, 0);
            var uDegree = _loader.GetIntArgument();

            _loader.MoveToArgumentOffset(expressID, 1);
            var vDegree = _loader.GetIntArgument();

            _loader.MoveToArgumentOffset(expressID, 2);
            var ctrlPointGroups = _loader.GetSetListArgument();
            foreach (var set in ctrlPointGroups)
            {
                var list = new List<Vec3>();
                foreach (var token in set)
                {
                    var pointID = _loader.GetRefArgument(token);
                    list.Add(_geometryLoader.GetCartesianPoint3D(pointID));
                }

                controlPts.Add(list);
            }

            _loader.MoveToArgumentOffset(expressID, 3);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 4);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 5);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 6);
            _ = _loader.GetStringArgument();

            _loader.MoveToArgumentOffset(expressID, 7);
            var knotSetU = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(expressID, 8);
            var knotSetV = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(expressID, 9);
            var indexesSetU = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(expressID, 10);
            var indexesSetV = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(expressID, 12);
            var weightPointGroups = _loader.GetSetListArgument();
            foreach (var set in weightPointGroups)
            {
                var list = new List<double>();
                foreach (var token in set)
                {
                    list.Add(_loader.GetDoubleArgument(token));
                }

                weightPts.Add(list);
            }

            foreach (var token in knotSetU)
            {
                uMultiplicity.Add((uint)_loader.GetIntArgument(token));
            }

            foreach (var token in knotSetV)
            {
                vMultiplicity.Add((uint)_loader.GetIntArgument(token));
            }

            foreach (var token in indexesSetU)
            {
                uKnots.Add(_loader.GetDoubleArgument(token));
            }

            foreach (var token in indexesSetV)
            {
                vKnots.Add(_loader.GetDoubleArgument(token));
            }

            surface.BSplineSurface.Active = true;
            surface.BSplineSurface.UDegree = uDegree;
            surface.BSplineSurface.VDegree = vDegree;
            surface.BSplineSurface.ControlPoints = controlPts;
            surface.BSplineSurface.UMultiplicity = uMultiplicity;
            surface.BSplineSurface.VMultiplicity = vMultiplicity;
            surface.BSplineSurface.UKnots = uKnots;
            surface.BSplineSurface.VKnots = vKnots;
            surface.BSplineSurface.WeightPoints = weightPts;

            return surface;
        }

        if (lineType == Type("IFCCYLINDRICALSURFACE"))
        {
            var surface = new IfcSurface();

            _loader.MoveToArgumentOffset(expressID, 0);
            var locationID = _loader.GetRefArgument();
            surface.transformation = _geometryLoader.GetLocalPlacement(locationID);

            _loader.MoveToArgumentOffset(expressID, 1);
            var radius = _loader.GetDoubleArgument();

            surface.CylinderSurface.Active = true;
            surface.CylinderSurface.Radius = radius;

            return surface;
        }

        if (lineType == Type("IFCSURFACEOFREVOLUTION"))
        {
            var surface = new IfcSurface();

            _loader.MoveToArgumentOffset(expressID, 0);
            var profileID = _loader.GetRefArgument();
            var profile = _geometryLoader.GetProfile3D(profileID);

            _loader.MoveToArgumentOffset(expressID, 1);
            if (_loader.GetTokenType() == IfcTokenType.Ref)
            {
                _loader.StepBack();
                var placementID = _loader.GetRefArgument();
                surface.transformation = _geometryLoader.GetLocalPlacement(placementID);
            }

            _loader.MoveToArgumentOffset(expressID, 2);
            var locationID = _loader.GetRefArgument();

            surface.RevolutionSurface.Active = true;
            surface.RevolutionSurface.Direction = _geometryLoader.GetLocalPlacement(locationID);
            surface.RevolutionSurface.Profile = profile;

            return surface;
        }

        if (lineType == Type("IFCSURFACEOFLINEAREXTRUSION"))
        {
            var surface = new IfcSurface();

            _loader.MoveToArgumentOffset(expressID, 0);
            var profileID = _loader.GetRefArgument();
            var profile = _geometryLoader.GetProfile(profileID);

            _loader.MoveToArgumentOffset(expressID, 2);
            var directionID = _loader.GetRefArgument();
            var direction = _geometryLoader.GetCartesianPoint3D(directionID);

            _loader.MoveToArgumentOffset(expressID, 3);
            var length = 0.0;
            if (_loader.GetTokenType() == IfcTokenType.Real)
            {
                _loader.StepBack();
                length = _loader.GetDoubleArgument();
            }

            surface.ExtrusionSurface.Active = true;
            surface.ExtrusionSurface.Length = length;
            surface.ExtrusionSurface.Profile = profile;
            surface.ExtrusionSurface.Direction = direction;

            _loader.MoveToArgumentOffset(expressID, 1);
            var locationID = _loader.GetRefArgument();
            surface.transformation = _geometryLoader.GetLocalPlacement(locationID);

            return surface;
        }

        return new IfcSurface();
    }

    public IfcFlatMesh GetFlatMesh(uint expressID, bool applyLinearScalingFactor = true)
    {
        var flatMesh = new IfcFlatMesh
        {
            expressID = expressID
        };

        var composedMesh = GetMesh(expressID);

        var mat = GeometryUtils.FlattenTransformation(Identity4);
        if (applyLinearScalingFactor)
        {
            var scale = _geometryLoader.GetLinearScalingFactor();
            mat = ScaleMatrix(scale);
        }

        var color = new Vec4(1, 1, 1, 1);
        var hasColor = false;

        var normalizeIfc = IfcRepresentationConstants.NormalizeIFC;
        var combined = MatrixMultiply4(MatrixMultiply4(_transformation, normalizeIfc), mat);
        AddComposedMeshToFlatMesh(flatMesh, composedMesh, combined, color, hasColor);

        return flatMesh;
    }

    private void AddComposedMeshToFlatMesh(
        IfcFlatMesh flatMesh,
        IfcComposedMesh composedMesh,
        double[] parentMatrix,
        Vec4 color,
        bool hasColor)
    {
        var newParentColor = color;
        var newHasColor = hasColor;
        var newMatrix = MatrixMultiply4(parentMatrix, composedMesh.transformation);

        if (composedMesh.hasColor && !hasColor)
        {
            newHasColor = true;
            newParentColor = composedMesh.color;
        }

        if (composedMesh.hasGeometry)
        {
            var geometry = new IfcPlacedGeometry();

            if (!_isCoordinated && _settings._coordinateToOrigin)
            {
                if (_expressIDToGeometry.TryGetValue(composedMesh.expressID, out var geomOrigin) && geomOrigin.numPoints > 0)
                {
                    var pt = geomOrigin.GetPoint(0);
                    var transformedPt = TransformPoint(newMatrix, pt);
                    _coordinationMatrix = TranslationMatrix(transformedPt * -1.0);
                    _isCoordinated = true;
                }
            }

            if (!_expressIDToGeometry.TryGetValue(composedMesh.expressID, out var geom))
            {
                return;
            }

            if (geom.isPolygon && !_settings._exportPolylines)
            {
                return;
            }

            if (geometry.testReverse())
            {
                geom.ReverseFaces();
            }

            var translation = geom.Normalize();
            _expressIDToGeometry[composedMesh.expressID] = geom;

            if (!composedMesh.hasColor)
            {
                geometry.color = newParentColor;
            }
            else
            {
                geometry.color = composedMesh.color;
                newParentColor = composedMesh.color;
                newHasColor = composedMesh.hasColor;
            }

            geometry.transformation = MatrixMultiply4(MatrixMultiply4(_coordinationMatrix, newMatrix), translation);
            geometry.SetFlatTransformation();
            geometry.geometryExpressID = composedMesh.expressID;

            flatMesh.geometries.Add(geometry);
        }
        else if (composedMesh.hasColor)
        {
            newParentColor = composedMesh.color;
            newHasColor = composedMesh.hasColor;
        }

        foreach (var child in composedMesh.children)
        {
            AddComposedMeshToFlatMesh(flatMesh, child, newMatrix, newParentColor, newHasColor);
        }
    }

    public IfcGeometry BoolProcess(IReadOnlyList<IfcGeometry> firstGeoms, List<IfcGeometry> secondGeoms, string op, IfcGeometrySettings settings)
    {
        return _boolEngine.BoolProcess(firstGeoms, secondGeoms, op, settings);
    }

    private void ApplyBooleanToMeshChildren(
        IfcComposedMesh composedMesh,
        List<IfcGeometry> secondGroups,
        string op,
        IfcGeometrySettings settings,
        double[] mat)
    {
        var newMat = MatrixMultiply4(mat, composedMesh.transformation);
        var transformationBreaksWinding = GeometryUtils.MatrixFlipsTriangles(newMat);

        if (composedMesh.hasGeometry && _expressIDToGeometry.TryGetValue(composedMesh.expressID, out var meshGeom))
        {
            var transformedGeoms = GeometryUtils.transformIfcGeometry(meshGeom, newMat, transformationBreaksWinding);
            var geometryResult = BoolProcess(transformedGeoms, secondGroups, op, settings);

            var invMat = InvertMatrix4(newMat);
            transformationBreaksWinding = GeometryUtils.MatrixFlipsTriangles(invMat);

            var localGeom = GeometryUtils.transformIfcGeometry(geometryResult, invMat, transformationBreaksWinding);
            var localGeomMerged = new IfcGeometry();
            foreach (var geom in localGeom)
            {
                localGeomMerged.MergeGeometry(geom);
            }

            _expressIDToGeometry[composedMesh.expressID] = localGeomMerged;
        }

        foreach (var child in composedMesh.children)
        {
            ApplyBooleanToMeshChildren(child, secondGroups, op, settings, newMat);
        }
    }

    private List<uint> Read2DArrayOfThreeIndices()
    {
        var result = new List<uint>();

        _ = _loader.GetTokenType();

        while (_loader.GetTokenType() == IfcTokenType.SetBegin)
        {
            result.Add((uint)_loader.GetIntArgument());
            result.Add((uint)_loader.GetIntArgument());
            result.Add((uint)_loader.GetIntArgument());

            _ = _loader.GetTokenType();
        }

        return result;
    }

    private void ReadIndexedPolygonalFace(uint expressID, List<IfcBound3D> bounds, IReadOnlyList<Vec3> points)
    {
        var lineType = _loader.GetLineType(expressID);

        bounds.Add(new IfcBound3D());

        if (lineType == Type("IFCINDEXEDPOLYGONALFACEWITHVOIDS") ||
            lineType == Type("IFCINDEXEDPOLYGONALFACE"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var indexIDs = _loader.GetSetArgument();

            foreach (var indexID in indexIDs)
            {
                var index = (uint)_loader.GetIntArgument(indexID);
                var point = points[(int)index - 1];
                bounds[^1].curve.points.Add(point);
            }

            if (lineType == Type("IFCINDEXEDPOLYGONALFACE"))
            {
                return;
            }

            _loader.MoveToArgumentOffset(expressID, 1);
            _ = _loader.GetTokenType();

            while (_loader.GetTokenType() == IfcTokenType.SetBegin)
            {
                bounds.Add(new IfcBound3D());

                while (_loader.GetTokenType() != IfcTokenType.SetEnd)
                {
                    _loader.StepBack();
                    var index = (uint)_loader.GetIntArgument();
                    var point = points[(int)index - 1];
                    bounds[^1].curve.points.Add(point);
                }
            }

            return;
        }
    }

    private IfcGeometry GetBrep(uint expressID)
    {
        var lineType = _loader.GetLineType(expressID);

        if (lineType == Type("IFCCONNECTEDFACESET") ||
            lineType == Type("IFCCLOSEDSHELL") ||
            lineType == Type("IFCOPENSHELL"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var faces = _loader.GetSetArgument();

            var geometry = new IfcGeometry();
            foreach (var faceToken in faces)
            {
                var faceID = _loader.GetRefArgument(faceToken);
                AddFaceToGeometry(faceID, geometry);
            }

            return geometry;
        }

        return new IfcGeometry();
    }

    private void AddFaceToGeometry(uint expressID, IfcGeometry geometry)
    {
        var lineType = _loader.GetLineType(expressID);

        if (lineType == Type("IFCFACE"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var bounds = _loader.GetSetArgument();

            var bounds3D = new List<IfcBound3D>(bounds.Count);
            foreach (var bound in bounds)
            {
                var boundID = _loader.GetRefArgument(bound);
                bounds3D.Add(_geometryLoader.GetBound(boundID));
            }

            GeometryUtils.TriangulateBounds(geometry, bounds3D, expressID);
            return;
        }

        if (lineType == Type("IFCADVANCEDFACE"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var bounds = _loader.GetSetArgument();

            var bounds3D = new List<IfcBound3D>(bounds.Count);
            foreach (var bound in bounds)
            {
                var boundID = _loader.GetRefArgument(bound);
                bounds3D.Add(_geometryLoader.GetBound(boundID));
            }

            _loader.MoveToArgumentOffset(expressID, 1);
            var surfRef = _loader.GetRefArgument();

            var surface = GetSurface(surfRef);

            if (surface.BSplineSurface.Active)
            {
                TriangulateBspline(geometry, bounds3D, surface, _geometryLoader.GetLinearScalingFactor());
            }
            else if (surface.CylinderSurface.Active)
            {
                TriangulateCylindricalSurface(geometry, bounds3D, surface, _settings._circleSegments);
            }
            else if (surface.RevolutionSurface.Active)
            {
                TriangulateRevolution(geometry, bounds3D, surface, _settings._circleSegments);
            }
            else if (surface.ExtrusionSurface.Active)
            {
                TriangulateExtrusion(geometry, bounds3D, surface);
            }
            else
            {
                GeometryUtils.TriangulateBounds(geometry, bounds3D, expressID);
            }
        }
    }

    public IfcGeometryProcessor Clone(IfcLoader newLoader)
    {
        ArgumentNullException.ThrowIfNull(newLoader);

        return new IfcGeometryProcessor(
            _settings,
            new Dictionary<uint, IfcGeometry>(_expressIDToGeometry),
            _geometryLoader.Clone(newLoader),
            _transformation,
            newLoader,
            _boolEngine,
            _schemaManager,
            _isCoordinated,
            _expressIdCyl,
            _expressIdRect,
            _coordinationMatrix,
            _predefinedCylinder,
            _predefinedCube);
    }

    // TODO: complete parity with src/cpp/web-ifc/geometry/operations/mesh_utils.h implementations.
    private static void TriangulateBspline(IfcGeometry geometry, List<IfcBound3D> bounds, IfcSurface surface, double linearScale)
    {
        _ = surface;
        _ = linearScale;
        GeometryUtils.TriangulateBounds(geometry, bounds, 0);
    }

    // TODO: complete parity with src/cpp/web-ifc/geometry/operations/mesh_utils.h implementations.
    private static void TriangulateCylindricalSurface(IfcGeometry geometry, List<IfcBound3D> bounds, IfcSurface surface, double numRots)
    {
        _ = surface;
        _ = numRots;
        GeometryUtils.TriangulateBounds(geometry, bounds, 0);
    }

    // TODO: complete parity with src/cpp/web-ifc/geometry/operations/mesh_utils.h implementations.
    private static void TriangulateRevolution(IfcGeometry geometry, List<IfcBound3D> bounds, IfcSurface surface, double numRots)
    {
        _ = surface;
        _ = numRots;
        GeometryUtils.TriangulateBounds(geometry, bounds, 0);
    }

    // TODO: complete parity with src/cpp/web-ifc/geometry/operations/mesh_utils.h implementations.
    private static void TriangulateExtrusion(IfcGeometry geometry, List<IfcBound3D> bounds, IfcSurface surface)
    {
        _ = surface;
        GeometryUtils.TriangulateBounds(geometry, bounds, 0);
    }

    private uint Type(string typeName)
    {
        if (_typeCodeCache.TryGetValue(typeName, out var cached))
        {
            return cached;
        }

        var value = _schemaManager.IfcTypeToTypeCode(typeName);
        _typeCodeCache[typeName] = value;
        return value;
    }

    private static Vec3 GetMatrixColumn(double[] matrix, int column)
    {
        var offset = column * 4;
        return new Vec3(matrix[offset + 0], matrix[offset + 1], matrix[offset + 2]);
    }

    private static Vec3 GetMatrixTranslation(double[] matrix)
    {
        return new Vec3(matrix[12], matrix[13], matrix[14]);
    }

    private static double[] TranslationMatrix(Vec3 translation)
    {
        return new double[]
        {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            translation.X, translation.Y, translation.Z, 1
        };
    }

    private static double[] ScaleMatrix(double uniform)
    {
        return new double[]
        {
            uniform, 0, 0, 0,
            0, uniform, 0, 0,
            0, 0, uniform, 0,
            0, 0, 0, 1
        };
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

    private static Vec3 TransformPoint(double[] matrix, Vec3 point)
    {
        var x = matrix[0] * point.X + matrix[4] * point.Y + matrix[8] * point.Z + matrix[12];
        var y = matrix[1] * point.X + matrix[5] * point.Y + matrix[9] * point.Z + matrix[13];
        var z = matrix[2] * point.X + matrix[6] * point.Y + matrix[10] * point.Z + matrix[14];
        var w = matrix[3] * point.X + matrix[7] * point.Y + matrix[11] * point.Z + matrix[15];

        if (Math.Abs(w) > BimEps.EPS_NONZERO && Math.Abs(w - 1.0) > BimEps.EPS_NONZERO)
        {
            return new Vec3(x / w, y / w, z / w);
        }

        return new Vec3(x, y, z);
    }

    private static Vec3 TransformDirection(double[] matrix, Vec3 direction)
    {
        var x = matrix[0] * direction.X + matrix[4] * direction.Y + matrix[8] * direction.Z;
        var y = matrix[1] * direction.X + matrix[5] * direction.Y + matrix[9] * direction.Z;
        var z = matrix[2] * direction.X + matrix[6] * direction.Y + matrix[10] * direction.Z;
        return new Vec3(x, y, z);
    }

    private static double[] InvertMatrix4(double[] matrix)
    {
        var augmented = new double[4, 8];

        for (var row = 0; row < 4; row++)
        {
            for (var col = 0; col < 4; col++)
            {
                augmented[row, col] = matrix[col * 4 + row];
            }

            for (var col = 0; col < 4; col++)
            {
                augmented[row, 4 + col] = row == col ? 1.0 : 0.0;
            }
        }

        for (var col = 0; col < 4; col++)
        {
            var pivotRow = col;
            var pivotAbs = Math.Abs(augmented[pivotRow, col]);
            for (var row = col + 1; row < 4; row++)
            {
                var testAbs = Math.Abs(augmented[row, col]);
                if (testAbs > pivotAbs)
                {
                    pivotAbs = testAbs;
                    pivotRow = row;
                }
            }

            if (pivotAbs <= BimEps.EPS_NONZERO)
            {
                return GeometryUtils.FlattenTransformation(Identity4);
            }

            if (pivotRow != col)
            {
                for (var i = 0; i < 8; i++)
                {
                    (augmented[col, i], augmented[pivotRow, i]) = (augmented[pivotRow, i], augmented[col, i]);
                }
            }

            var pivot = augmented[col, col];
            for (var i = 0; i < 8; i++)
            {
                augmented[col, i] /= pivot;
            }

            for (var row = 0; row < 4; row++)
            {
                if (row == col)
                {
                    continue;
                }

                var factor = augmented[row, col];
                if (Math.Abs(factor) <= BimEps.EPS_NONZERO)
                {
                    continue;
                }

                for (var i = 0; i < 8; i++)
                {
                    augmented[row, i] -= factor * augmented[col, i];
                }
            }
        }

        var inverse = new double[16];
        for (var row = 0; row < 4; row++)
        {
            for (var col = 0; col < 4; col++)
            {
                inverse[col * 4 + row] = augmented[row, 4 + col];
            }
        }

        return inverse;
    }

    private static void FlipFaceWinding(IfcGeometry geom)
    {
        for (uint i = 0; i < geom.numFaces; i++)
        {
            var idx = (int)(i * 3);
            (geom.indexData[idx + 0], geom.indexData[idx + 1]) = (geom.indexData[idx + 1], geom.indexData[idx + 0]);
        }
    }
}
