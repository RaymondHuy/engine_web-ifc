namespace BazanCDE.Parsing;

public sealed class ModelManager : IDisposable
{
    private readonly IIfcSchemaManager _schemaManager;
    private readonly List<IfcLoader?> _loaders = new();
    private readonly List<LoaderSettings> _settings = new();
    private readonly Dictionary<uint, IfcGeometryProcessor> _geometryProcessors = new();
    private readonly bool _mtEnabled;

    private bool _headerShown;

    public ModelManager(bool mtEnabled, IIfcSchemaManager? schemaManager = null)
    {
        _mtEnabled = mtEnabled;
        _schemaManager = schemaManager ?? new DefaultIfcSchemaManager();
    }

    public bool IsMultiThreadingEnabled => _mtEnabled;

    public IfcGeometryProcessor? GetGeometryProcessor(uint modelId)
    {
        if (!IsModelOpen(modelId))
        {
            return null;
        }

        if (!_geometryProcessors.TryGetValue(modelId, out var processor) || processor is null)
        {
            var loader = GetIfcLoader(modelId);
            if (loader is null)
            {
                return null;
            }

            var setting = GetSettings(modelId);
            processor = new IfcGeometryProcessor(
                loader,
                _schemaManager,
                setting.CircleSegments,
                setting.CoordinateToOrigin,
                setting.TolerancePlaneIntersection,
                setting.TolerancePlaneDeviation,
                setting.ToleranceBackDeviationDistance,
                setting.ToleranceInsideOutsidePerimeter,
                setting.ToleranceScalarEquality,
                setting.PlaneRefitIterations,
                setting.BooleanUnionThreshold
            );

            _geometryProcessors[modelId] = processor;
        }

        return processor;
    }

    public LoaderSettings GetSettings(uint modelId)
    {
        if (!IsModelOpen(modelId) || modelId >= _settings.Count)
        {
            return new LoaderSettings();
        }

        return _settings[(int)modelId];
    }

    public IfcLoader? GetIfcLoader(uint modelId)
    {
        if (!IsModelOpen(modelId))
        {
            return null;
        }

        return _loaders[(int)modelId];
    }

    public IIfcSchemaManager GetSchemaManager()
    {
        return _schemaManager;
    }

    public bool IsModelOpen(uint modelId)
    {
        if (modelId >= _loaders.Count)
        {
            return false;
        }

        return _loaders[(int)modelId] is not null;
    }

    public void CloseModel(uint modelId)
    {
        if (!IsModelOpen(modelId))
        {
            return;
        }

        if (_geometryProcessors.Remove(modelId, out var processor) && processor is not null)
        {
            processor.Clear();
        }

        _loaders[(int)modelId]?.Dispose();
        _loaders[(int)modelId] = null;
    }

    public uint CreateModel(LoaderSettings? settings = null)
    {
        var effectiveSettings = settings ?? new LoaderSettings();

        if (!_headerShown)
        {
            _headerShown = true;
        }

        var loader = new IfcLoader(
            effectiveSettings.TapeSize,
            effectiveSettings.MemoryLimit,
            effectiveSettings.LineWriterBuffer,
            _schemaManager
        );

        _loaders.Add(loader);
        _settings.Add(effectiveSettings);

        return (uint)(_loaders.Count - 1);
    }

    public void CloseAllModels()
    {
        foreach (var entry in _geometryProcessors)
        {
            entry.Value?.Clear();
        }

        _geometryProcessors.Clear();

        for (var i = 0; i < _loaders.Count; i++)
        {
            _loaders[i]?.Dispose();
            _loaders[i] = null;
        }

        _loaders.Clear();
        _settings.Clear();
    }

    public void Dispose()
    {
        CloseAllModels();
    }
}
