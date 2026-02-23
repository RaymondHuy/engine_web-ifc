using System.Numerics;

namespace BazanCDE.Parsing;

public sealed record IfcParsedModel(
    int MeshCount,
    int TriangleCount,
    IReadOnlyList<IfcParsedMesh> Meshes
);

public sealed record IfcParsedMesh(
    uint ExpressId,
    float[] Positions,
    uint[] Indices,
    IfcParsedColor Color
);

public sealed record IfcParsedColor(float R, float G, float B, float A)
{
    public Vector4 ToVector4() => new(R, G, B, A);
}

public sealed record IfcPlacedGeometry(
    uint GeometryExpressId,
    Vector4 Color,
    Matrix4x4 Transformation,
    float[] Positions,
    uint[] Indices)
{
    public float[] FlatTransformation =>
    [
        Transformation.M11, Transformation.M12, Transformation.M13, Transformation.M14,
        Transformation.M21, Transformation.M22, Transformation.M23, Transformation.M24,
        Transformation.M31, Transformation.M32, Transformation.M33, Transformation.M34,
        Transformation.M41, Transformation.M42, Transformation.M43, Transformation.M44
    ];
}

public sealed record IfcFlatMesh(
    uint ExpressId,
    IReadOnlyList<IfcPlacedGeometry> Geometries
);

public sealed record IfcComposedMesh(
    uint ExpressId,
    bool HasGeometry,
    bool HasColor,
    Vector4 Color,
    Matrix4x4 Transformation,
    IReadOnlyList<IfcComposedMesh> Children
);
