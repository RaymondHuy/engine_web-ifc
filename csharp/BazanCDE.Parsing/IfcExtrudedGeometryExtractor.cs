using System.Numerics;

namespace BazanCDE.Parsing;

// Temporary compatibility stub for CLI debug/parity modes.
// TODO: replace with full port of the original extractor.
public sealed class IfcExtrudedGeometryExtractor
{
    private readonly IfcLoader _loader;
    private readonly IIfcSchemaManager _schemaManager;

    public IfcExtrudedGeometryExtractor(IfcLoader loader, IIfcSchemaManager schemaManager)
    {
        _loader = loader;
        _schemaManager = schemaManager;
    }

    public bool TryBuildMesh(uint expressId, out object? mesh, out int triangleCount)
    {
        _ = expressId;
        _ = _loader;
        _ = _schemaManager;

        mesh = null;
        triangleCount = 0;
        return false;
    }

    public bool TryGetExtrudedProfileLoopCounts(uint expressId, out int outerCount, out IReadOnlyList<int> holeCounts)
    {
        _ = expressId;

        outerCount = 0;
        holeCounts = Array.Empty<int>();
        return false;
    }

    public bool TryGetCurve2DPointCount(uint expressId, out int pointCount)
    {
        _ = expressId;

        pointCount = 0;
        return false;
    }

    public bool TryGetCurve2DPoints(uint expressId, out List<Vector2> points)
    {
        _ = expressId;

        points = new List<Vector2>();
        return false;
    }
}
