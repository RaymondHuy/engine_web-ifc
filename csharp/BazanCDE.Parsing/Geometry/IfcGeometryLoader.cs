using BazanCDE.Parsing.Geometry.Operations;
using BazanCDE.Parsing.Geometry.Operations.BimGeometry;
using BazanCDE.Parsing.Geometry.Representation;
using BazanCDE.Parsing.Utilities;

namespace BazanCDE.Parsing.Geometry;

public sealed class IfcGeometryLoader
{
    private readonly IfcLoader _loader;
    private readonly IIfcSchemaManager _schemaManager;

    private Dictionary<uint, List<uint>> _relVoids;
    private Dictionary<uint, List<uint>> _relNests;
    private Dictionary<uint, List<uint>> _relAggregates;
    private Dictionary<uint, List<(uint ItemId, uint StyleId)>> _styledItems;
    private Dictionary<uint, List<(uint ObjectId, uint MaterialId)>> _relMaterials;
    private Dictionary<uint, List<(uint MaterialId, uint DefinitionId)>> _materialDefinitions;

    private double _linearScalingFactor = 1.0;
    private double _squaredScalingFactor = 1.0;
    private double _cubicScalingFactor = 1.0;
    private double _angularScalingFactor = 1.0;
    private string _angleUnits = "RADIAN";
    private readonly ushort _circleSegments;

    private readonly Dictionary<uint, IfcCurve> _localCurveCache = new();
    private readonly Dictionary<uint, Vec3> _cartesianPoint3DCache = new();
    private readonly Dictionary<uint, Vec2> _cartesianPoint2DCache = new();
    private readonly Dictionary<uint, Mat4> _expressIDToPlacement = new();

    private readonly uint _ifcCartesianPoint;
    private readonly uint _ifcDirection;
    private readonly uint _ifcCartesianPointList2D;
    private readonly uint _ifcCartesianPointList3D;
    private readonly uint _ifcLocalPlacement;
    private readonly uint _ifcAxis2Placement2D;
    private readonly uint _ifcAxis2Placement3D;
    private readonly uint _ifcAxis1Placement;
    private readonly uint _ifcPolyline;
    private readonly uint _ifcLine;
    private readonly uint _ifcEdgeCurve;
    private readonly uint _ifcOrientedEdge;
    private readonly uint _ifcVertexPoint;
    private readonly uint _ifcEdgeLoop;
    private readonly uint _ifcPolyLoop;
    private readonly uint _ifcFaceBound;
    private readonly uint _ifcFaceOuterBound;
    private readonly uint _ifcCompositeCurve;
    private readonly uint _ifcCompositeCurveSegment;
    private readonly uint _ifcTrimmedCurve;
    private readonly uint _ifcSectionedSolid;
    private readonly uint _ifcSectionedSolidHorizontal;
    private readonly uint _ifcSectionedSurface;

    public IfcGeometryLoader(
        IfcLoader loader,
        IIfcSchemaManager schemaManager,
        ushort circleSegments,
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

        _circleSegments = circleSegments;

        _relVoids = PopulateRelVoidsMap();
        _relNests = PopulateRelNestsMap();
        _relAggregates = PopulateRelAggregatesMap();
        _styledItems = PopulateStyledItemMap();
        _relMaterials = PopulateRelMaterialsMap();
        _materialDefinitions = PopulateMaterialDefinitionsMap();

        _ifcCartesianPoint = Type("IFCCARTESIANPOINT");
        _ifcDirection = Type("IFCDIRECTION");
        _ifcCartesianPointList2D = Type("IFCCARTESIANPOINTLIST2D");
        _ifcCartesianPointList3D = Type("IFCCARTESIANPOINTLIST3D");
        _ifcLocalPlacement = Type("IFCLOCALPLACEMENT");
        _ifcAxis2Placement2D = Type("IFCAXIS2PLACEMENT2D");
        _ifcAxis2Placement3D = Type("IFCAXIS2PLACEMENT3D");
        _ifcAxis1Placement = Type("IFCAXIS1PLACEMENT");
        _ifcPolyline = Type("IFCPOLYLINE");
        _ifcLine = Type("IFCLINE");
        _ifcEdgeCurve = Type("IFCEDGECURVE");
        _ifcOrientedEdge = Type("IFCORIENTEDEDGE");
        _ifcVertexPoint = Type("IFCVERTEXPOINT");
        _ifcEdgeLoop = Type("IFCEDGELOOP");
        _ifcPolyLoop = Type("IFCPOLYLOOP");
        _ifcFaceBound = Type("IFCFACEBOUND");
        _ifcFaceOuterBound = Type("IFCFACEOUTERBOUND");
        _ifcCompositeCurve = Type("IFCCOMPOSITECURVE");
        _ifcCompositeCurveSegment = Type("IFCCOMPOSITECURVESEGMENT");
        _ifcTrimmedCurve = Type("IFCTRIMMEDCURVE");
        _ifcSectionedSolid = Type("IFCSECTIONEDSOLID");
        _ifcSectionedSolidHorizontal = Type("IFCSECTIONEDSOLIDHORIZONTAL");
        _ifcSectionedSurface = Type("IFCSECTIONEDSURFACE");

        GeometryUtils.SetEpsilons(
            toleranceScalarEquality,
            planeRefitIterations,
            booleanUnionThreshold);

        _ = tolerancePlaneIntersection;
        _ = tolerancePlaneDeviation;
        _ = toleranceBackDeviationDistance;
        _ = toleranceInsideOutsidePerimeter;

        ReadLinearScalingFactor();
    }

    private IfcGeometryLoader(IfcLoader loader, IIfcSchemaManager schemaManager, IfcGeometryLoader source)
        : this(
            loader,
            schemaManager,
            source._circleSegments,
            1.0E-04,
            1.0E-04,
            1.0E-04,
            1.0E-10,
            Epsilons._TOLERANCE_SCALAR_EQUALITY,
            Epsilons._PLANE_REFIT_ITERATIONS,
            Epsilons._BOOLEAN_UNION_THRESHOLD)
    {
        _relVoids = CloneMap(source._relVoids);
        _relNests = CloneMap(source._relNests);
        _relAggregates = CloneMap(source._relAggregates);
        _styledItems = ClonePairMap(source._styledItems);
        _relMaterials = ClonePairMap(source._relMaterials);
        _materialDefinitions = ClonePairMap(source._materialDefinitions);

        _linearScalingFactor = source._linearScalingFactor;
        _squaredScalingFactor = source._squaredScalingFactor;
        _cubicScalingFactor = source._cubicScalingFactor;
        _angularScalingFactor = source._angularScalingFactor;
        _angleUnits = source._angleUnits;
    }

    public void ResetCache()
    {
        _relVoids = PopulateRelVoidsMap();
        _relNests = PopulateRelNestsMap();
        _relAggregates = PopulateRelAggregatesMap();
        _styledItems = PopulateStyledItemMap();
        _relMaterials = PopulateRelMaterialsMap();
        _materialDefinitions = PopulateMaterialDefinitionsMap();
    }

    public void Clear()
    {
        _expressIDToPlacement.Clear();
        _cartesianPoint3DCache.Clear();
        _cartesianPoint2DCache.Clear();
        _localCurveCache.Clear();
    }

    public Vec3[] GetAxis1Placement(uint expressID)
    {
        var location = new Vec3(0, 0, 0);
        var axis = new Vec3(0, 0, 1);

        if (_loader.GetLineType(expressID) != _ifcAxis1Placement)
        {
            return [location, axis];
        }

        _loader.MoveToArgumentOffset(expressID, 0);
        var locRef = _loader.GetOptionalRefArgument();
        if (locRef != 0)
        {
            location = GetCartesianPoint3D(locRef);
        }

        _loader.MoveToArgumentOffset(expressID, 1);
        var axisRef = _loader.GetOptionalRefArgument();
        if (axisRef != 0)
        {
            var dir = GetCartesianPoint3D(axisRef);
            if (dir.Length() > IfcRepresentationConstants.EPS_TINY)
            {
                axis = Vec3.Normalize(dir);
            }
        }

        return [location, axis];
    }

    public double[] GetAxis2Placement2D(uint expressID)
    {
        var m = Identity3();

        if (_loader.GetLineType(expressID) != _ifcAxis2Placement2D)
        {
            return m;
        }

        var location = new Vec2(0, 0);
        var xAxis = new Vec2(1, 0);

        _loader.MoveToArgumentOffset(expressID, 0);
        var locationRef = _loader.GetOptionalRefArgument();
        if (locationRef != 0)
        {
            location = GetCartesianPoint2D(locationRef);
        }

        _loader.MoveToArgumentOffset(expressID, 1);
        var refDirRef = _loader.GetOptionalRefArgument();
        if (refDirRef != 0)
        {
            var d = GetCartesianPoint2D(refDirRef);
            if (d.Length() > IfcRepresentationConstants.EPS_TINY)
            {
                xAxis = Vec2.Normalize(d);
            }
        }

        var yAxis = new Vec2(-xAxis.Y, xAxis.X);

        m[0] = xAxis.X;
        m[1] = xAxis.Y;
        m[2] = 0;

        m[3] = yAxis.X;
        m[4] = yAxis.Y;
        m[5] = 0;

        m[6] = location.X;
        m[7] = location.Y;
        m[8] = 1;

        return m;
    }

    public Mat4 GetLocalPlacement(uint expressID, Vec3? vector = null)
    {
        if (_expressIDToPlacement.TryGetValue(expressID, out var cached))
        {
            return cached;
        }

        var lineType = _loader.GetLineType(expressID);

        if (lineType == _ifcLocalPlacement)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var parentRef = _loader.GetOptionalRefArgument();

            _loader.MoveToArgumentOffset(expressID, 1);
            var relPlacementRef = _loader.GetOptionalRefArgument();

            var local = relPlacementRef != 0 ? GetPlacementMatrix(relPlacementRef, vector) : Mat4.Identity;
            var result = parentRef != 0 ? GetLocalPlacement(parentRef, vector) * local : local;
            _expressIDToPlacement[expressID] = result;
            return result;
        }

        var direct = GetPlacementMatrix(expressID, vector);
        _expressIDToPlacement[expressID] = direct;
        return direct;
    }

    public Vec3 GetCartesianPoint3D(uint expressID)
    {
        if (_cartesianPoint3DCache.TryGetValue(expressID, out var cached))
        {
            return cached;
        }

        var result = new Vec3(0, 0, 0);
        var lineType = _loader.GetLineType(expressID);

        if (lineType == _ifcCartesianPoint || lineType == _ifcDirection)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            if (_loader.GetTokenType() == IfcTokenType.SetBegin)
            {
                _loader.StepBack();
                var args = _loader.GetSetArgument();
                if (args.Count > 0)
                {
                    var x = GetNumber(args[0]);
                    var y = args.Count > 1 ? GetNumber(args[1]) : 0.0;
                    var z = args.Count > 2 ? GetNumber(args[2]) : 0.0;
                    result = new Vec3(x, y, z);
                }
            }
        }

        _cartesianPoint3DCache[expressID] = result;
        return result;
    }

    public Vec2 GetCartesianPoint2D(uint expressID)
    {
        if (_cartesianPoint2DCache.TryGetValue(expressID, out var cached))
        {
            return cached;
        }

        var result = new Vec2(0, 0);
        var lineType = _loader.GetLineType(expressID);

        if (lineType == _ifcCartesianPoint || lineType == _ifcDirection)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            if (_loader.GetTokenType() == IfcTokenType.SetBegin)
            {
                _loader.StepBack();
                var args = _loader.GetSetArgument();
                if (args.Count > 0)
                {
                    var x = GetNumber(args[0]);
                    var y = args.Count > 1 ? GetNumber(args[1]) : 0.0;
                    result = new Vec2(x, y);
                }
            }
        }

        _cartesianPoint2DCache[expressID] = result;
        return result;
    }

    public Vec3 GetVector(uint expressID)
    {
        var lineType = _loader.GetLineType(expressID);

        if (lineType == Type("IFCVECTOR"))
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var orientationRef = _loader.GetOptionalRefArgument();
            var dir = orientationRef != 0 ? GetCartesianPoint3D(orientationRef) : new Vec3(1, 0, 0);

            _loader.MoveToArgumentOffset(expressID, 1);
            var mag = ReadNumericAtCurrentToken();
            return dir * mag;
        }

        if (lineType == _ifcDirection)
        {
            return GetCartesianPoint3D(expressID);
        }

        return new Vec3(0, 0, 0);
    }

    public IfcProfile GetProfile(uint expressID)
    {
        var profile = new IfcProfile
        {
            type = _schemaManager.IfcTypeCodeToType(_loader.GetLineType(expressID))
        };

        var lineType = _loader.GetLineType(expressID);

        if (lineType == Type("IFCARBITRARYCLOSEDPROFILEDEF"))
        {
            _loader.MoveToArgumentOffset(expressID, 2);
            var outerCurveRef = _loader.GetOptionalRefArgument();
            if (outerCurveRef != 0)
            {
                profile.curve = GetCurve(outerCurveRef, 2);
            }
            profile.isConvex = CurveUtils.IsCurveConvex(profile.curve);
            return profile;
        }

        if (lineType == Type("IFCARBITRARYPROFILEDEFWITHVOIDS"))
        {
            _loader.MoveToArgumentOffset(expressID, 2);
            var outerCurveRef = _loader.GetOptionalRefArgument();
            if (outerCurveRef != 0)
            {
                profile.curve = GetCurve(outerCurveRef, 2);
            }

            _loader.MoveToArgumentOffset(expressID, 3);
            var holeRefs = _loader.GetSetArgument();
            for (var i = 0; i < holeRefs.Count; i++)
            {
                var holeRef = _loader.GetRefArgument(holeRefs[i]);
                if (holeRef != 0)
                {
                    profile.holes.Add(GetCurve(holeRef, 2));
                }
            }

            profile.isConvex = CurveUtils.IsCurveConvex(profile.curve);
            return profile;
        }

        return GetProfileByLine(expressID);
    }

    public IfcProfile GetProfile3D(uint expressID)
    {
        return GetProfile(expressID);
    }

    public IfcCurve GetLocalCurve(uint expressID)
    {
        if (_localCurveCache.TryGetValue(expressID, out var cached))
        {
            return cached;
        }

        var curve = GetCurve(expressID, 3);
        _localCurveCache[expressID] = curve;
        return curve;
    }

    public IfcCurve GetCurve(uint expressID, byte dimensions, bool edge = false)
    {
        var curve = new IfcCurve();
        var parameters = new ComputeCurveParams
        {
            dimensions = dimensions,
            edge = edge
        };

        ComputeCurve(expressID, curve, parameters);
        return curve;
    }

    public double ComputeCurveLength(IfcCurve curve)
    {
        if (curve.points.Count < 2)
        {
            return 0.0;
        }

        var total = 0.0;
        for (var i = 1; i < curve.points.Count; i++)
        {
            total += (curve.points[i] - curve.points[i - 1]).Length();
        }

        return total;
    }

    public double ComputeLengthToPoint(IfcCurve curve, Vec3 targetPoint)
    {
        if (curve.points.Count < 2)
        {
            return 0.0;
        }

        var total = 0.0;
        for (var i = 1; i < curve.points.Count; i++)
        {
            var p0 = curve.points[i - 1];
            var p1 = curve.points[i];
            var seg = p1 - p0;
            var segLen = seg.Length();
            if (segLen <= IfcRepresentationConstants.EPS_TINY)
            {
                continue;
            }

            var t = Vec3.Dot(targetPoint - p0, seg) / (segLen * segLen);
            if (t >= 0.0 && t <= 1.0)
            {
                var projected = p0 + seg * t;
                if ((projected - targetPoint).Length() <= IfcRepresentationConstants.EPS_SMALL)
                {
                    return total + segLen * t;
                }
            }

            total += segLen;
        }

        return total;
    }

    public double GetParameterForPoint(IfcCurve curve, double totalLength, Vec3 point)
    {
        if (totalLength <= IfcRepresentationConstants.EPS_TINY)
        {
            return 0.0;
        }

        var lengthToPoint = ComputeLengthToPoint(curve, point);
        return Math.Clamp(lengthToPoint / totalLength, 0.0, 1.0);
    }

    public bool ReadIfcCartesianPointList(uint expressID)
    {
        var lineType = _loader.GetLineType(expressID);
        return lineType == _ifcCartesianPointList2D || lineType == _ifcCartesianPointList3D;
    }

    public List<Vec3> ReadIfcCartesianPointList3D(uint expressID)
    {
        var points = new List<Vec3>();

        if (_loader.GetLineType(expressID) != _ifcCartesianPointList3D)
        {
            return points;
        }

        _loader.MoveToArgumentOffset(expressID, 0);
        var listOffsets = _loader.GetSetListArgument();
        for (var i = 0; i < listOffsets.Count; i++)
        {
            var row = listOffsets[i];
            if (row.Count == 0)
            {
                continue;
            }

            var x = GetNumber(row[0]);
            var y = row.Count > 1 ? GetNumber(row[1]) : 0.0;
            var z = row.Count > 2 ? GetNumber(row[2]) : 0.0;
            points.Add(new Vec3(x, y, z));
        }

        return points;
    }

    public List<Vec2> ReadIfcCartesianPointList2D(uint expressID)
    {
        var points = new List<Vec2>();

        if (_loader.GetLineType(expressID) != _ifcCartesianPointList2D)
        {
            return points;
        }

        _loader.MoveToArgumentOffset(expressID, 0);
        var listOffsets = _loader.GetSetListArgument();
        for (var i = 0; i < listOffsets.Count; i++)
        {
            var row = listOffsets[i];
            if (row.Count == 0)
            {
                continue;
            }

            var x = GetNumber(row[0]);
            var y = row.Count > 1 ? GetNumber(row[1]) : 0.0;
            points.Add(new Vec2(x, y));
        }

        return points;
    }

    public IfcCurve GetOrientedEdge(uint expressID)
    {
        if (_loader.GetLineType(expressID) != _ifcOrientedEdge)
        {
            return new IfcCurve();
        }

        _loader.MoveToArgumentOffset(expressID, 3);
        var edgeRef = _loader.GetOptionalRefArgument();

        _loader.MoveToArgumentOffset(expressID, 4);
        var orientation = ReadBoolAtCurrentToken(defaultValue: true);

        var curve = edgeRef != 0 ? GetEdge(edgeRef) : new IfcCurve();
        if (!orientation)
        {
            curve.points.Reverse();
        }

        return curve;
    }

    public IfcCurve GetEdge(uint expressID)
    {
        var lineType = _loader.GetLineType(expressID);
        if (lineType != _ifcEdgeCurve)
        {
            return new IfcCurve();
        }

        _loader.MoveToArgumentOffset(expressID, 0);
        var startVertex = _loader.GetOptionalRefArgument();

        _loader.MoveToArgumentOffset(expressID, 1);
        var endVertex = _loader.GetOptionalRefArgument();

        _loader.MoveToArgumentOffset(expressID, 2);
        var edgeGeometry = _loader.GetOptionalRefArgument();

        _loader.MoveToArgumentOffset(expressID, 3);
        var sameSense = ReadBoolAtCurrentToken(defaultValue: true);

        IfcCurve curve;
        if (edgeGeometry != 0)
        {
            curve = GetCurve(edgeGeometry, 3, edge: true);
        }
        else
        {
            curve = new IfcCurve();
            if (startVertex != 0)
            {
                curve.Add(GetVertexPoint(startVertex));
            }

            if (endVertex != 0)
            {
                curve.Add(GetVertexPoint(endVertex));
            }
        }

        if (!sameSense)
        {
            curve.points.Reverse();
        }

        return curve;
    }

    public IfcBound3D GetBound(uint expressID)
    {
        var bound = new IfcBound3D
        {
            type = _loader.GetLineType(expressID) == _ifcFaceOuterBound
                ? IfcBoundType.OUTERBOUND
                : IfcBoundType.BOUND,
            orientation = true,
            curve = new IfcCurve()
        };

        _loader.MoveToArgumentOffset(expressID, 0);
        var loopRef = _loader.GetOptionalRefArgument();

        _loader.MoveToArgumentOffset(expressID, 1);
        bound.orientation = ReadBoolAtCurrentToken(defaultValue: true);

        if (loopRef != 0)
        {
            bound.curve = GetLoop(loopRef);
            if (!bound.orientation)
            {
                bound.curve.points.Reverse();
            }
        }

        return bound;
    }

    public IfcCurve GetLoop(uint expressID)
    {
        var loop = new IfcCurve();
        var lineType = _loader.GetLineType(expressID);

        if (lineType == _ifcPolyLoop)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var pointRefs = _loader.GetSetArgument();
            for (var i = 0; i < pointRefs.Count; i++)
            {
                var pointRef = _loader.GetRefArgument(pointRefs[i]);
                if (pointRef != 0)
                {
                    loop.Add(GetCartesianPoint3D(pointRef), removeCoincident: false);
                }
            }

            if (loop.points.Count > 0 && (loop.points[0] - loop.points[^1]).Length() > IfcRepresentationConstants.EPS_TINY)
            {
                loop.Add(loop.points[0], removeCoincident: false);
            }

            return loop;
        }

        if (lineType == _ifcEdgeLoop)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var edgeRefs = _loader.GetSetArgument();
            for (var i = 0; i < edgeRefs.Count; i++)
            {
                var edgeRef = _loader.GetRefArgument(edgeRefs[i]);
                if (edgeRef == 0)
                {
                    continue;
                }

                var edge = GetOrientedEdge(edgeRef);
                for (var p = 0; p < edge.points.Count; p++)
                {
                    loop.Add(edge.points[p]);
                }
            }

            return loop;
        }

        return loop;
    }

    public Vec4? GetColor(uint expressID)
    {
        _ = expressID;
        return null;
    }

    public IfcCrossSections GetCrossSections2D(uint expressID)
    {
        var sections = new IfcCrossSections();
        var lineType = _loader.GetLineType(expressID);

        if (lineType == _ifcSectionedSolid || lineType == _ifcSectionedSolidHorizontal)
        {
            _loader.MoveToArgumentOffset(expressID, 1);
            var faces = _loader.GetSetArgument();
            for (var i = 0; i < faces.Count; i++)
            {
                var faceRef = _loader.GetRefArgument(faces[i]);
                if (faceRef == 0)
                {
                    continue;
                }

                var profile = GetProfile(faceRef);
                sections.curves.Add(profile.curve);
                sections.expressID.Add(faceRef);
            }

            return sections;
        }

        if (lineType == _ifcSectionedSurface)
        {
            _loader.MoveToArgumentOffset(expressID, 2);
            var faces = _loader.GetSetArgument();
            for (var i = 0; i < faces.Count; i++)
            {
                var faceRef = _loader.GetRefArgument(faces[i]);
                if (faceRef == 0)
                {
                    continue;
                }

                var profile = GetProfile(faceRef);
                sections.curves.Add(profile.curve);
                sections.expressID.Add(faceRef);
            }
        }

        return sections;
    }

    public void getPlacementsOnCurvePoints(uint curveID, SortedDictionary<double, double[]> mapPlacements)
    {
        if (mapPlacements.Count < 2)
        {
            return;
        }

        var minDistance = mapPlacements.Keys.First();
        var maxDistance = mapPlacements.Keys.Last();

        var basisCurve = GetLocalCurve(curveID);
        if (basisCurve.points.Count == 0)
        {
            return;
        }

        var keys = mapPlacements.Keys.ToList();
        for (var i = 0; i < keys.Count; i++)
        {
            var k = keys[i];
            mapPlacements[k] = basisCurve.getPlacementAtDistance(k, IfcCurve.CurvePlacementMode.TangentAsZAxis);
        }

        if (minDistance <= 0)
        {
            mapPlacements[0] = basisCurve.getPlacementAtDistance(0, IfcCurve.CurvePlacementMode.TangentAsZAxis);
        }

        var sumLength = 0.0;
        var previousPoint = basisCurve.points[0];
        for (var ii = 1; ii < basisCurve.points.Count; ii++)
        {
            var point1 = basisCurve.points[ii];
            var length = (point1 - previousPoint).Length();
            sumLength += length;

            if (sumLength > maxDistance)
            {
                break;
            }

            if (sumLength >= minDistance)
            {
                mapPlacements[sumLength] = basisCurve.getPlacementAtDistance(sumLength, IfcCurve.CurvePlacementMode.TangentAsZAxis);
            }

            previousPoint = point1;
        }

        if (maxDistance >= sumLength)
        {
            mapPlacements[sumLength] = basisCurve.getPlacementAtDistance(sumLength, IfcCurve.CurvePlacementMode.TangentAsZAxis);
        }
    }

    public IfcCrossSections GetCrossSections3D(uint expressID, bool scaled = false, double[]? coordination = null)
    {
        _ = scaled;
        _ = coordination;
        return GetCrossSections2D(expressID);
    }

    public IfcAlignment GetAlignment(uint expressID, IfcAlignment? alignment = null, double[]? transform = null, uint sourceExpressID = uint.MaxValue)
    {
        _ = expressID;
        _ = transform;
        _ = sourceExpressID;
        return alignment ?? new IfcAlignment();
    }

    public bool GetColor(uint expressID, out Vec4 outputColor)
    {
        var color = GetColor(expressID);
        if (color.HasValue)
        {
            outputColor = color.Value;
            return true;
        }

        outputColor = new Vec4(1, 1, 1, 1);
        return false;
    }

    public IReadOnlyDictionary<uint, List<uint>> GetRelVoids()
    {
        return _relVoids;
    }

    public IReadOnlyDictionary<uint, List<(uint ItemId, uint StyleId)>> GetStyledItems()
    {
        return _styledItems;
    }

    public IReadOnlyDictionary<uint, List<(uint ObjectId, uint MaterialId)>> GetRelMaterials()
    {
        return _relMaterials;
    }

    public IReadOnlyDictionary<uint, List<(uint MaterialId, uint DefinitionId)>> GetMaterialDefinitions()
    {
        return _materialDefinitions;
    }

    public double GetLinearScalingFactor()
    {
        return _linearScalingFactor;
    }

    public string GetAngleUnits()
    {
        return _angleUnits;
    }

    public IfcGeometryLoader Clone(IfcLoader newLoader)
    {
        if (newLoader is null)
        {
            throw new ArgumentNullException(nameof(newLoader));
        }

        return new IfcGeometryLoader(newLoader, _schemaManager, this);
    }

    private IfcCurve GetAlignmentCurve(uint expressID, uint parentExpressID = uint.MaxValue)
    {
        _ = expressID;
        _ = parentExpressID;
        uint lineType = _loader.GetLineType(expressID);
        IfcCurve alignmentCurve;
        if (lineType == Type("IFCALIGNMENTSEGMENT"))
        {
            _loader.MoveToArgumentOffset(expressID, 5);
            uint localPlacement = 0;
            if (_loader.GetTokenType() == IfcTokenType.Ref)
            {
                _loader.StepBack();
                localPlacement = _loader.GetRefArgument();
            }
            Mat4 transform_t = Mat4.Identity;
            if (localPlacement != 0 && _loader.IsValidExpressID(localPlacement))
            {
                transform_t = GetLocalPlacement(localPlacement);
            }
        }
        throw new NotImplementedException("IfcGeometryLoader.GetAlignmentCurve is not fully ported yet.");
    }

    private IfcProfile GetProfileByLine(uint expressID)
    {
        var profile = new IfcProfile
        {
            type = _schemaManager.IfcTypeCodeToType(_loader.GetLineType(expressID))
        };

        var lineType = _loader.GetLineType(expressID);

        if (lineType == Type("IFCRECTANGLEPROFILEDEF"))
        {
            _loader.MoveToArgumentOffset(expressID, 2);
            var placementRef = _loader.GetOptionalRefArgument();
            var placement = placementRef != 0 ? GetAxis2Placement2D(placementRef) : Identity3();

            _loader.MoveToArgumentOffset(expressID, 3);
            var xdim = ReadNumericAtCurrentToken();
            _loader.MoveToArgumentOffset(expressID, 4);
            var ydim = ReadNumericAtCurrentToken();

            profile.curve = CurveUtils.GetRectangleCurve(xdim, ydim, placement, _circleSegments, 0);
            profile.isConvex = true;
            return profile;
        }

        if (lineType == Type("IFCCIRCLEPROFILEDEF"))
        {
            _loader.MoveToArgumentOffset(expressID, 2);
            var placementRef = _loader.GetOptionalRefArgument();
            var placement = placementRef != 0 ? GetAxis2Placement2D(placementRef) : Identity3();

            _loader.MoveToArgumentOffset(expressID, 3);
            var r = ReadNumericAtCurrentToken();

            profile.curve = CurveUtils.GetCircleCurve((float)r, _circleSegments, placement);
            profile.isConvex = true;
            return profile;
        }

        if (lineType == Type("IFCELLIPSEPROFILEDEF"))
        {
            _loader.MoveToArgumentOffset(expressID, 2);
            var placementRef = _loader.GetOptionalRefArgument();
            var placement = placementRef != 0 ? GetAxis2Placement2D(placementRef) : Identity3();

            _loader.MoveToArgumentOffset(expressID, 3);
            var rx = ReadNumericAtCurrentToken();
            _loader.MoveToArgumentOffset(expressID, 4);
            var ry = ReadNumericAtCurrentToken();

            profile.curve = CurveUtils.GetEllipseCurve((float)rx, (float)ry, _circleSegments, placement);
            profile.isConvex = true;
            return profile;
        }

        return profile;
    }

    private Vec3 GetVertexPoint(uint expressID)
    {
        if (_loader.GetLineType(expressID) == _ifcVertexPoint)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var pointRef = _loader.GetOptionalRefArgument();
            if (pointRef != 0)
            {
                return GetCartesianPoint3D(pointRef);
            }
        }

        return GetCartesianPoint3D(expressID);
    }

    private IfcTrimmingSelect GetTrimSelect(uint dim, List<uint> tapeOffsets)
    {
        _ = dim;
        var trim = new IfcTrimmingSelect();

        if (tapeOffsets.Count == 0)
        {
            return trim;
        }

        var t = _loader.GetTokenType(tapeOffsets[0]);
        switch (t)
        {
            case IfcTokenType.Real:
            case IfcTokenType.Integer:
                trim.trimType = IfcTrimmingSelectType.TRIM_BY_PARAMETER;
                trim.value = GetNumber(tapeOffsets[0]);
                break;
            case IfcTokenType.Ref:
            {
                var pointRef = _loader.GetRefArgument(tapeOffsets[0]);
                if (pointRef != 0)
                {
                    if (dim == 2)
                    {
                        trim.trimType = IfcTrimmingSelectType.TRIM_BY_POSITION;
                        trim.pos = GetCartesianPoint2D(pointRef);
                    }
                    else
                    {
                        trim.trimType = IfcTrimmingSelectType.TRIM_BY_POSITION;
                        trim.pos3D = GetCartesianPoint3D(pointRef);
                    }
                }

                break;
            }
        }

        return trim;
    }

    private sealed class ComputeCurveParams
    {
        public byte dimensions = 2;
        public bool ignorePlacement = false;
        public bool edge = false;
        public int sameSense = -1;
        public bool hasTrim = false;
        public IfcTrimmingSelect trimStart = new();
        public IfcTrimmingSelect trimEnd = new();
        public TrimSense trimSense = TrimSense.TRIM_SENSE_SAME;
    }

    private void ComputeCurve(uint expressID, IfcCurve curve, ComputeCurveParams parameters)
    {
        var lineType = _loader.GetLineType(expressID);

        if (lineType == _ifcPolyline)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var points = _loader.GetSetArgument();
            for (var i = 0; i < points.Count; i++)
            {
                var pRef = _loader.GetRefArgument(points[i]);
                if (pRef == 0)
                {
                    continue;
                }

                curve.Add(parameters.dimensions == 2
                    ? new Vec3(GetCartesianPoint2D(pRef).X, GetCartesianPoint2D(pRef).Y, 0)
                    : GetCartesianPoint3D(pRef), removeCoincident: !parameters.edge);
            }

            return;
        }

        if (lineType == _ifcLine)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var pRef = _loader.GetOptionalRefArgument();
            _loader.MoveToArgumentOffset(expressID, 1);
            var vRef = _loader.GetOptionalRefArgument();

            var p = pRef != 0 ? GetCartesianPoint3D(pRef) : new Vec3(0, 0, 0);
            var v = vRef != 0 ? GetVector(vRef) : new Vec3(1, 0, 0);

            curve.Add(p, removeCoincident: false);
            curve.Add(p + v, removeCoincident: false);
            return;
        }

        if (lineType == _ifcTrimmedCurve)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var basisRef = _loader.GetOptionalRefArgument();
            if (basisRef != 0)
            {
                var basis = GetCurve(basisRef, parameters.dimensions, parameters.edge);
                curve.points.AddRange(basis.points);
            }

            return;
        }

        if (lineType == _ifcCompositeCurve)
        {
            _loader.MoveToArgumentOffset(expressID, 0);
            var segRefs = _loader.GetSetArgument();
            for (var i = 0; i < segRefs.Count; i++)
            {
                var segRef = _loader.GetRefArgument(segRefs[i]);
                if (segRef == 0)
                {
                    continue;
                }

                var segType = _loader.GetLineType(segRef);
                if (segType == _ifcCompositeCurveSegment)
                {
                    _loader.MoveToArgumentOffset(segRef, 2);
                    var parentCurveRef = _loader.GetOptionalRefArgument();
                    var segCurve = parentCurveRef != 0
                        ? GetCurve(parentCurveRef, parameters.dimensions)
                        : new IfcCurve();

                    _loader.MoveToArgumentOffset(segRef, 1);
                    var sameSense = ReadBoolAtCurrentToken(defaultValue: true);
                    if (!sameSense)
                    {
                        segCurve.points.Reverse();
                    }

                    for (var p = 0; p < segCurve.points.Count; p++)
                    {
                        curve.Add(segCurve.points[p]);
                    }
                }
                else
                {
                    var segCurve = GetCurve(segRef, parameters.dimensions);
                    for (var p = 0; p < segCurve.points.Count; p++)
                    {
                        curve.Add(segCurve.points[p]);
                    }
                }
            }

            return;
        }

        if (lineType == _ifcOrientedEdge)
        {
            var edgeCurve = GetOrientedEdge(expressID);
            curve.points.AddRange(edgeCurve.points);
            return;
        }

        if (lineType == _ifcEdgeCurve)
        {
            var edgeCurve = GetEdge(expressID);
            curve.points.AddRange(edgeCurve.points);
            return;
        }

        throw new NotImplementedException($"IfcGeometryLoader.ComputeCurve: line type '{_schemaManager.IfcTypeCodeToType(lineType)}' is not ported yet.");
    }

    private void convertAngleUnits(ref double degrees, ref double rad)
    {
        if (_angleUnits.Equals("RADIAN", StringComparison.OrdinalIgnoreCase))
        {
            rad = degrees;
            degrees = degrees * 180.0 / IfcRepresentationConstants.CONST_PI;
            return;
        }

        rad = GeometryUtils.angleConversion(degrees, _angleUnits);
    }

    private double ReadLenghtMeasure()
    {
        return ReadNumericAtCurrentToken();
    }

    private void ReadCurveMeasureSelect(IfcTrimmingSelect trim)
    {
        if (trim is null)
        {
            return;
        }

        trim.trimType = IfcTrimmingSelectType.TRIM_BY_LENGTH;
        trim.value = ReadLenghtMeasure();
    }

    private List<IfcSegmentIndexSelect> ReadCurveIndices()
    {
        var indices = new List<IfcSegmentIndexSelect>();
        var sets = _loader.GetSetListArgument();

        for (var i = 0; i < sets.Count; i++)
        {
            var item = new IfcSegmentIndexSelect();
            var list = sets[i];
            for (var j = 0; j < list.Count; j++)
            {
                var tk = _loader.GetTokenType(list[j]);
                if (tk == IfcTokenType.Label)
                {
                    item.type = _loader.GetStringArgument();
                }
                else if (tk == IfcTokenType.Integer)
                {
                    item.indexs.Add((uint)Math.Max(0, _loader.GetIntArgument(list[j])));
                }
            }

            indices.Add(item);
        }

        return indices;
    }

    private Dictionary<uint, List<uint>> PopulateRelVoidsMap()
    {
        var map = new Dictionary<uint, List<uint>>();
        var relVoidsType = Type("IFCRELVOIDSELEMENT");
        var lines = _loader.GetExpressIDsWithType(relVoidsType);

        for (var i = 0; i < lines.Count; i++)
        {
            var id = lines[i];
            _loader.MoveToArgumentOffset(id, 4);
            var host = _loader.GetOptionalRefArgument();
            _loader.MoveToArgumentOffset(id, 5);
            var opening = _loader.GetOptionalRefArgument();

            if (host == 0 || opening == 0)
            {
                continue;
            }

            if (!map.TryGetValue(host, out var list))
            {
                list = new List<uint>();
                map[host] = list;
            }

            list.Add(opening);
        }

        return map;
    }

    private Dictionary<uint, List<uint>> PopulateRelNestsMap()
    {
        var map = new Dictionary<uint, List<uint>>();
        var relType = Type("IFCRELNESTS");
        var lines = _loader.GetExpressIDsWithType(relType);

        for (var i = 0; i < lines.Count; i++)
        {
            var id = lines[i];
            _loader.MoveToArgumentOffset(id, 4);
            var parent = _loader.GetOptionalRefArgument();
            _loader.MoveToArgumentOffset(id, 5);
            var childrenOffsets = _loader.GetSetArgument();

            if (parent == 0 || childrenOffsets.Count == 0)
            {
                continue;
            }

            if (!map.TryGetValue(parent, out var list))
            {
                list = new List<uint>();
                map[parent] = list;
            }

            for (var j = 0; j < childrenOffsets.Count; j++)
            {
                var child = _loader.GetRefArgument(childrenOffsets[j]);
                if (child != 0)
                {
                    list.Add(child);
                }
            }
        }

        return map;
    }

    private Dictionary<uint, List<uint>> PopulateRelAggregatesMap()
    {
        var map = new Dictionary<uint, List<uint>>();
        var relType = Type("IFCRELAGGREGATES");
        var lines = _loader.GetExpressIDsWithType(relType);

        for (var i = 0; i < lines.Count; i++)
        {
            var id = lines[i];
            _loader.MoveToArgumentOffset(id, 4);
            var parent = _loader.GetOptionalRefArgument();
            _loader.MoveToArgumentOffset(id, 5);
            var childrenOffsets = _loader.GetSetArgument();

            if (parent == 0 || childrenOffsets.Count == 0)
            {
                continue;
            }

            if (!map.TryGetValue(parent, out var list))
            {
                list = new List<uint>();
                map[parent] = list;
            }

            for (var j = 0; j < childrenOffsets.Count; j++)
            {
                var child = _loader.GetRefArgument(childrenOffsets[j]);
                if (child != 0)
                {
                    list.Add(child);
                }
            }
        }

        return map;
    }

    private Dictionary<uint, List<(uint ItemId, uint StyleId)>> PopulateStyledItemMap()
    {
        var map = new Dictionary<uint, List<(uint ItemId, uint StyleId)>>();
        var styledItemType = Type("IFCSTYLEDITEM");
        var lines = _loader.GetExpressIDsWithType(styledItemType);

        for (var i = 0; i < lines.Count; i++)
        {
            var id = lines[i];
            _loader.MoveToArgumentOffset(id, 0);
            var item = _loader.GetOptionalRefArgument();
            if (item == 0)
            {
                continue;
            }

            _loader.MoveToArgumentOffset(id, 1);
            var styles = _loader.GetSetArgument();
            if (styles.Count == 0)
            {
                continue;
            }

            if (!map.TryGetValue(item, out var list))
            {
                list = new List<(uint, uint)>();
                map[item] = list;
            }

            for (var j = 0; j < styles.Count; j++)
            {
                var style = _loader.GetRefArgument(styles[j]);
                if (style != 0)
                {
                    list.Add((item, style));
                }
            }
        }

        return map;
    }

    private Dictionary<uint, List<(uint ObjectId, uint MaterialId)>> PopulateRelMaterialsMap()
    {
        var map = new Dictionary<uint, List<(uint ObjectId, uint MaterialId)>>();
        var relType = Type("IFCRELASSOCIATESMATERIAL");
        var lines = _loader.GetExpressIDsWithType(relType);

        for (var i = 0; i < lines.Count; i++)
        {
            var id = lines[i];

            _loader.MoveToArgumentOffset(id, 4);
            var objects = _loader.GetSetArgument();

            _loader.MoveToArgumentOffset(id, 5);
            var material = _loader.GetOptionalRefArgument();

            if (material == 0 || objects.Count == 0)
            {
                continue;
            }

            for (var j = 0; j < objects.Count; j++)
            {
                var obj = _loader.GetRefArgument(objects[j]);
                if (obj == 0)
                {
                    continue;
                }

                if (!map.TryGetValue(obj, out var list))
                {
                    list = new List<(uint, uint)>();
                    map[obj] = list;
                }

                list.Add((obj, material));
            }
        }

        return map;
    }

    private Dictionary<uint, List<(uint MaterialId, uint DefinitionId)>> PopulateMaterialDefinitionsMap()
    {
        // The full IFC material-definition graph is large; keep this map available and fill later.
        return new Dictionary<uint, List<(uint MaterialId, uint DefinitionId)>>();
    }

    private void ReadLinearScalingFactor()
    {
        _linearScalingFactor = 1.0;
        _squaredScalingFactor = 1.0;
        _cubicScalingFactor = 1.0;
        _angularScalingFactor = 1.0;
        _angleUnits = "RADIAN";

        var unitAssignmentType = Type("IFCUNITASSIGNMENT");
        var siUnitType = Type("IFCSIUNIT");
        var conversionBasedUnitType = Type("IFCCONVERSIONBASEDUNIT");

        var unitAssignments = _loader.GetExpressIDsWithType(unitAssignmentType);
        if (unitAssignments.Count == 0)
        {
            return;
        }

        for (var i = 0; i < unitAssignments.Count; i++)
        {
            _loader.MoveToArgumentOffset(unitAssignments[i], 0);
            var units = _loader.GetSetArgument();

            for (var j = 0; j < units.Count; j++)
            {
                var unitRef = _loader.GetRefArgument(units[j]);
                if (unitRef == 0)
                {
                    continue;
                }

                var unitType = _loader.GetLineType(unitRef);

                if (unitType == siUnitType)
                {
                    _loader.MoveToArgumentOffset(unitRef, 0);
                    var dimToken = _loader.GetTokenType();
                    if (dimToken != IfcTokenType.Enum)
                    {
                        continue;
                    }

                    _loader.StepBack();
                    var dimension = _loader.GetStringArgument();

                    _loader.MoveToArgumentOffset(unitRef, 1);
                    var prefixToken = _loader.GetTokenType();
                    var prefix = string.Empty;
                    if (prefixToken == IfcTokenType.Enum)
                    {
                        _loader.StepBack();
                        prefix = _loader.GetStringArgument();
                    }

                    _loader.MoveToArgumentOffset(unitRef, 2);
                    var nameToken = _loader.GetTokenType();
                    var name = string.Empty;
                    if (nameToken == IfcTokenType.Enum)
                    {
                        _loader.StepBack();
                        name = _loader.GetStringArgument();
                    }

                    var factor = ConvertPrefix(prefix);

                    if (dimension.Equals("LENGTHUNIT", StringComparison.OrdinalIgnoreCase) &&
                        name.Equals("METRE", StringComparison.OrdinalIgnoreCase))
                    {
                        _linearScalingFactor = factor;
                        _squaredScalingFactor = factor * factor;
                        _cubicScalingFactor = factor * factor * factor;
                    }
                    else if (dimension.Equals("PLANEANGLEUNIT", StringComparison.OrdinalIgnoreCase))
                    {
                        _angleUnits = name.Equals("RADIAN", StringComparison.OrdinalIgnoreCase) ? "RADIAN" : "DEGREE";
                        _angularScalingFactor = _angleUnits == "RADIAN"
                            ? 1.0
                            : IfcRepresentationConstants.CONST_PI / 180.0;
                    }
                }
                else if (unitType == conversionBasedUnitType)
                {
                    // Fallback path; we preserve default 1.0 until a full conversion parser is ported.
                }
            }
        }
    }

    private double ConvertPrefix(string prefix)
    {
        if (string.IsNullOrWhiteSpace(prefix))
        {
            return 1.0;
        }

        return prefix.ToUpperInvariant() switch
        {
            "EXA" => 1E18,
            "PETA" => 1E15,
            "TERA" => 1E12,
            "GIGA" => 1E9,
            "MEGA" => 1E6,
            "KILO" => 1E3,
            "HECTO" => 1E2,
            "DECA" => 1E1,
            "DECI" => 1E-1,
            "CENTI" => 1E-2,
            "MILLI" => 1E-3,
            "MICRO" => 1E-6,
            "NANO" => 1E-9,
            "PICO" => 1E-12,
            "FEMTO" => 1E-15,
            "ATTO" => 1E-18,
            _ => 1.0
        };
    }

    private uint Type(string name)
    {
        return _schemaManager.IfcTypeToTypeCode(name);
    }

    private double GetNumber(uint tapeOffset)
    {
        var token = _loader.GetTokenType(tapeOffset);
        if (token == IfcTokenType.Real)
        {
            return _loader.GetDoubleArgument(tapeOffset);
        }

        if (token == IfcTokenType.Integer)
        {
            return _loader.GetIntArgument(tapeOffset);
        }

        return 0.0;
    }

    private double ReadNumericAtCurrentToken()
    {
        var token = _loader.GetTokenType();
        if (token == IfcTokenType.Real)
        {
            _loader.StepBack();
            return _loader.GetDoubleArgument();
        }

        if (token == IfcTokenType.Integer)
        {
            _loader.StepBack();
            return _loader.GetIntArgument();
        }

        return 0.0;
    }

    private bool ReadBoolAtCurrentToken(bool defaultValue)
    {
        var tk = _loader.GetTokenType();
        if (tk == IfcTokenType.Enum || tk == IfcTokenType.Label)
        {
            _loader.StepBack();
            var str = _loader.GetStringArgument();
            if (str.Equals("T", StringComparison.OrdinalIgnoreCase) ||
                str.Equals("TRUE", StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }

            if (str.Equals("F", StringComparison.OrdinalIgnoreCase) ||
                str.Equals("FALSE", StringComparison.OrdinalIgnoreCase))
            {
                return false;
            }
        }

        if (tk == IfcTokenType.Integer)
        {
            _loader.StepBack();
            return _loader.GetIntArgument() != 0;
        }

        return defaultValue;
    }

    private Mat4 GetPlacementMatrix(uint expressID, Vec3? vector)
    {
        var lineType = _loader.GetLineType(expressID);

        if (lineType == _ifcAxis2Placement3D)
        {
            var origin = new Vec3(0, 0, 0);
            var zAxis = vector ?? new Vec3(0, 0, 1);
            var xAxis = new Vec3(1, 0, 0);

            _loader.MoveToArgumentOffset(expressID, 0);
            var locationRef = _loader.GetOptionalRefArgument();
            if (locationRef != 0)
            {
                origin = GetCartesianPoint3D(locationRef);
            }

            _loader.MoveToArgumentOffset(expressID, 1);
            var axisRef = _loader.GetOptionalRefArgument();
            if (axisRef != 0)
            {
                var axis = GetCartesianPoint3D(axisRef);
                if (axis.Length() > IfcRepresentationConstants.EPS_TINY)
                {
                    zAxis = Vec3.Normalize(axis);
                }
            }

            _loader.MoveToArgumentOffset(expressID, 2);
            var refDirRef = _loader.GetOptionalRefArgument();
            if (refDirRef != 0)
            {
                var refDir = GetCartesianPoint3D(refDirRef);
                if (refDir.Length() > IfcRepresentationConstants.EPS_TINY)
                {
                    xAxis = Vec3.Normalize(refDir);
                }
            }

            var yAxis = Vec3.Cross(zAxis, xAxis);
            if (yAxis.Length() <= IfcRepresentationConstants.EPS_TINY)
            {
                xAxis = Math.Abs(zAxis.Z) > 0.9 ? new Vec3(1, 0, 0) : new Vec3(0, 0, 1);
                yAxis = Vec3.Cross(zAxis, xAxis);
            }

            yAxis = Vec3.Normalize(yAxis);
            xAxis = Vec3.Normalize(Vec3.Cross(yAxis, zAxis));

            return new Mat4(
                new Vec4(xAxis.X, xAxis.Y, xAxis.Z, 0),
                new Vec4(yAxis.X, yAxis.Y, yAxis.Z, 0),
                new Vec4(zAxis.X, zAxis.Y, zAxis.Z, 0),
                new Vec4(origin.X, origin.Y, origin.Z, 1));
        }

        if (lineType == _ifcAxis2Placement2D)
        {
            var m2 = GetAxis2Placement2D(expressID);
            return new Mat4(
                new Vec4(m2[0], m2[1], 0, 0),
                new Vec4(m2[3], m2[4], 0, 0),
                new Vec4(0, 0, 1, 0),
                new Vec4(m2[6], m2[7], 0, 1));
        }

        return Mat4.Identity;
    }

    private static double[] Identity3()
    {
        return
        [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        ];
    }

    private static Dictionary<uint, List<uint>> CloneMap(Dictionary<uint, List<uint>> input)
    {
        var clone = new Dictionary<uint, List<uint>>(input.Count);
        foreach (var kv in input)
        {
            clone[kv.Key] = new List<uint>(kv.Value);
        }

        return clone;
    }

    private static Dictionary<uint, List<(uint A, uint B)>> ClonePairMap<TA, TB>(Dictionary<uint, List<(TA A, TB B)>> input)
        where TA : unmanaged
        where TB : unmanaged
    {
        var clone = new Dictionary<uint, List<(uint, uint)>>(input.Count);
        foreach (var kv in input)
        {
            var list = new List<(uint, uint)>(kv.Value.Count);
            for (var i = 0; i < kv.Value.Count; i++)
            {
                list.Add((Convert.ToUInt32(kv.Value[i].A), Convert.ToUInt32(kv.Value[i].B)));
            }

            clone[kv.Key] = list;
        }

        return clone;
    }
}
