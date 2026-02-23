namespace BazanCDE.Parsing;

public sealed class LoaderSettings
{
    public bool CoordinateToOrigin { get; init; } = false;
    public ushort CircleSegments { get; init; } = 12;
    public uint TapeSize { get; init; } = 67_108_864;
    public uint MemoryLimit { get; init; } = 2_147_483_648;
    public ushort LineWriterBuffer { get; init; } = 10_000;

    public double TolerancePlaneIntersection { get; init; } = 1.0E-04;
    public double TolerancePlaneDeviation { get; init; } = 1.0E-04;
    public double ToleranceBackDeviationDistance { get; init; } = 1.0E-04;
    public double ToleranceInsideOutsidePerimeter { get; init; } = 1.0E-10;
    public double ToleranceScalarEquality { get; init; } = 1.0E-04;
    public ushort PlaneRefitIterations { get; init; } = 1;
    public ushort BooleanUnionThreshold { get; init; } = 150;
}
