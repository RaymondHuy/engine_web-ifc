using System.Numerics;

namespace BazanCDE.Parsing;

public sealed class IfcGeometrySettings
{
    public bool CoordinateToOrigin { get; set; } = false;
    public bool OptimizeProfiles { get; set; } = true;
    public bool ExportPolylines { get; set; } = false;
    public ushort CircleSegments { get; set; } = 12;

    public double TolerancePlaneIntersection { get; set; } = 1.0E-04;
    public double TolerancePlaneDeviation { get; set; } = 1.0E-04;
    public double ToleranceBackDeviationDistance { get; set; } = 1.0E-04;
    public double ToleranceInsideOutsidePerimeter { get; set; } = 1.0E-10;
    public double ToleranceBoundingBox { get; set; } = 1.0E-02;
    public ushort BooleanUnionThreshold { get; set; } = 150;
}

public sealed class IfcGeometryProcessor
{
    private readonly IfcLoader _loader;
    private readonly IIfcSchemaManager _schemaManager;
    private readonly IfcGeometrySettings _settings;
    private readonly IfcGeometryLoader _geometryLoader;

    private IfcExtrudedGeometryExtractor _extractor;
    private Matrix4x4 _coordinationMatrix = Matrix4x4.Identity;

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

        _settings = new IfcGeometrySettings
        {
            CircleSegments = circleSegments,
            CoordinateToOrigin = coordinateToOrigin,
            TolerancePlaneIntersection = tolerancePlaneIntersection,
            TolerancePlaneDeviation = tolerancePlaneDeviation,
            ToleranceBackDeviationDistance = toleranceBackDeviationDistance,
            ToleranceInsideOutsidePerimeter = toleranceInsideOutsidePerimeter,
            BooleanUnionThreshold = (ushort)Math.Clamp(booleanUnionThreshold, 0, ushort.MaxValue)
        };

        _ = toleranceScalarEquality;
        _ = planeRefitIterations;

        _geometryLoader = new IfcGeometryLoader(
            _loader,
            _schemaManager,
            circleSegments,
            tolerancePlaneIntersection,
            tolerancePlaneDeviation,
            toleranceBackDeviationDistance,
            toleranceInsideOutsidePerimeter,
            toleranceScalarEquality,
            planeRefitIterations,
            booleanUnionThreshold);

        _extractor = new IfcExtrudedGeometryExtractor(_loader, _schemaManager);
    }

    public IfcParsedModel AnalyzeModel(int maxMeshes = 1_500, int maxTriangles = 800_000)
    {
        return _extractor.Extract(maxMeshes, maxTriangles);
    }

    public IfcFlatMesh GetFlatMesh(uint expressId, bool applyLinearScalingFactor = true)
    {
        _ = applyLinearScalingFactor;

        if (!_extractor.TryBuildMesh(expressId, out var mesh, out _))
        {
            return new IfcFlatMesh(expressId, Array.Empty<IfcPlacedGeometry>());
        }

        var placed = new IfcPlacedGeometry(
            GeometryExpressId: mesh.ExpressId,
            Color: mesh.Color.ToVector4(),
            Transformation: Matrix4x4.Identity,
            Positions: mesh.Positions,
            Indices: mesh.Indices
        );

        return new IfcFlatMesh(expressId, new[] { placed });
    }

    public IfcComposedMesh GetMesh(uint expressId)
    {
        if (!_extractor.TryBuildMesh(expressId, out var mesh, out _))
        {
            return new IfcComposedMesh(
                ExpressId: expressId,
                HasGeometry: false,
                HasColor: false,
                Color: Vector4.One,
                Transformation: Matrix4x4.Identity,
                Children: Array.Empty<IfcComposedMesh>()
            );
        }

        return new IfcComposedMesh(
            ExpressId: mesh.ExpressId,
            HasGeometry: true,
            HasColor: true,
            Color: mesh.Color.ToVector4(),
            Transformation: Matrix4x4.Identity,
            Children: Array.Empty<IfcComposedMesh>()
        );
    }

    public void SetTransformation(IReadOnlyList<double> values)
    {
        if (values is null || values.Count != 16)
        {
            throw new ArgumentException("Transformation requires exactly 16 values.", nameof(values));
        }

        _coordinationMatrix = new Matrix4x4(
            (float)values[0], (float)values[1], (float)values[2], (float)values[3],
            (float)values[4], (float)values[5], (float)values[6], (float)values[7],
            (float)values[8], (float)values[9], (float)values[10], (float)values[11],
            (float)values[12], (float)values[13], (float)values[14], (float)values[15]
        );
    }

    public float[] GetFlatCoordinationMatrix()
    {
        return
        [
            _coordinationMatrix.M11, _coordinationMatrix.M12, _coordinationMatrix.M13, _coordinationMatrix.M14,
            _coordinationMatrix.M21, _coordinationMatrix.M22, _coordinationMatrix.M23, _coordinationMatrix.M24,
            _coordinationMatrix.M31, _coordinationMatrix.M32, _coordinationMatrix.M33, _coordinationMatrix.M34,
            _coordinationMatrix.M41, _coordinationMatrix.M42, _coordinationMatrix.M43, _coordinationMatrix.M44
        ];
    }

    public Matrix4x4 GetCoordinationMatrix()
    {
        return _coordinationMatrix;
    }

    public IfcGeometrySettings GetSettings()
    {
        return _settings;
    }

    public IfcGeometryLoader GetLoader()
    {
        return _geometryLoader;
    }

    public void Clear()
    {
        _geometryLoader.Clear();
        _extractor = new IfcExtrudedGeometryExtractor(_loader, _schemaManager);
    }

    public IfcGeometryProcessor Clone(IfcLoader loader)
    {
        if (loader is null)
        {
            throw new ArgumentNullException(nameof(loader));
        }

        var clone = new IfcGeometryProcessor(
            loader,
            _schemaManager,
            _settings.CircleSegments,
            _settings.CoordinateToOrigin,
            _settings.TolerancePlaneIntersection,
            _settings.TolerancePlaneDeviation,
            _settings.ToleranceBackDeviationDistance,
            _settings.ToleranceInsideOutsidePerimeter,
            toleranceScalarEquality: 1.0E-04,
            planeRefitIterations: 1,
            _settings.BooleanUnionThreshold
        );

        clone._coordinationMatrix = _coordinationMatrix;
        return clone;
    }
}
