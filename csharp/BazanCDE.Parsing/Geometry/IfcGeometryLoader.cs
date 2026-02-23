namespace BazanCDE.Parsing;

public sealed class IfcGeometryLoader
{
    private readonly IfcLoader _loader;
    private readonly IIfcSchemaManager _schemaManager;
    private readonly ushort _circleSegments;

    private readonly Dictionary<uint, List<uint>> _relVoids = new();
    private readonly Dictionary<uint, List<KeyValuePair<uint, uint>>> _styledItems = new();
    private readonly Dictionary<uint, List<KeyValuePair<uint, uint>>> _relMaterials = new();
    private readonly Dictionary<uint, List<KeyValuePair<uint, uint>>> _materialDefinitions = new();

    private double _linearScalingFactor = 1.0;
    private string _angleUnits = string.Empty;

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
        _loader = loader;
        _schemaManager = schemaManager;
        _circleSegments = circleSegments;

        _ = tolerancePlaneIntersection;
        _ = tolerancePlaneDeviation;
        _ = toleranceBackDeviationDistance;
        _ = toleranceInsideOutsidePerimeter;
        _ = toleranceScalarEquality;
        _ = planeRefitIterations;
        _ = booleanUnionThreshold;
    }

    public void ResetCache()
    {
        _relVoids.Clear();
        _styledItems.Clear();
        _relMaterials.Clear();
        _materialDefinitions.Clear();
    }

    public IReadOnlyDictionary<uint, List<uint>> GetRelVoids()
    {
        return _relVoids;
    }

    public IReadOnlyDictionary<uint, List<KeyValuePair<uint, uint>>> GetStyledItems()
    {
        return _styledItems;
    }

    public IReadOnlyDictionary<uint, List<KeyValuePair<uint, uint>>> GetRelMaterials()
    {
        return _relMaterials;
    }

    public IReadOnlyDictionary<uint, List<KeyValuePair<uint, uint>>> GetMaterialDefinitions()
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

    public void Clear()
    {
        ResetCache();
    }

    public IfcGeometryLoader Clone(IfcLoader loader)
    {
        return new IfcGeometryLoader(
            loader,
            _schemaManager,
            _circleSegments,
            tolerancePlaneIntersection: 1.0E-04,
            tolerancePlaneDeviation: 1.0E-04,
            toleranceBackDeviationDistance: 1.0E-04,
            toleranceInsideOutsidePerimeter: 1.0E-10,
            toleranceScalarEquality: 1.0E-04,
            planeRefitIterations: 1,
            booleanUnionThreshold: 150);
    }
}
