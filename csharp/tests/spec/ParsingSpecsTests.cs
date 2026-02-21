using BazanCDE.Parsing;
using Xunit;

namespace BazanCDE.Parsing.Specs;

public sealed class ParsingSpecsTests
{
    private const uint TapeSize = 64 * 1024 * 1024;
    private const ulong MemoryLimit = 2UL * 1024 * 1024 * 1024;
    private const uint LineWriterBuffer = 10_000;

    private static readonly string SamplePath = ResolveSampleIfcPath();
    private static readonly IIfcSchemaManager SchemaManager = new DefaultIfcSchemaManager();

    [Fact]
    public void CanLoadModelAndBasicMetadata()
    {
        using var loader = LoadSampleFromStream();

        Assert.Equal("IFC2X3", SchemaManager.GetSchemaName(loader.GetSchema()));
        Assert.Equal(6490, loader.GetAllLines().Count);
        Assert.Equal((uint)14315, loader.GetMaxExpressId());
    }

    [Fact]
    public void CanQueryTypeCounts()
    {
        using var loader = LoadSampleFromStream();

        var singleValueType = SchemaManager.IfcTypeToTypeCode("IFCPROPERTYSINGLEVALUE");
        var wallStandardCaseType = SchemaManager.IfcTypeToTypeCode("IFCWALLSTANDARDCASE");

        Assert.Equal(456, loader.GetExpressIDsWithType(singleValueType).Count);
        Assert.Equal(17, loader.GetExpressIDsWithType(wallStandardCaseType).Count);
    }

    [Fact]
    public void CanReadCoreArgumentsFromPropertySet244()
    {
        using var loader = LoadSampleFromStream();

        var psetType = SchemaManager.IfcTypeToTypeCode("IFCPROPERTYSET");
        Assert.Equal(psetType, loader.GetLineType(244));

        loader.MoveToLineArgument(244, 0);
        Assert.Equal("0uNK5AgoP1Vw6UlaHiS$iF", loader.GetStringArgument());

        loader.MoveToLineArgument(244, 1);
        Assert.Equal((uint)41, loader.GetRefArgument());

        loader.MoveToLineArgument(244, 2);
        Assert.Equal("Pset_ColumnCommon", loader.GetStringArgument());
    }

    [Fact]
    public void CanReadSetArgumentFromPropertySet244()
    {
        using var loader = LoadSampleFromStream();

        loader.MoveToLineArgument(244, 4);
        var setOffsets = loader.GetSetArgument();

        Assert.Equal(3, setOffsets.Count);

        var refs = setOffsets.Select(loader.GetRefArgument).ToArray();
        Assert.Equal(new uint[] { 241, 242, 243 }, refs);
    }

    [Fact]
    public void CanReadOptionalRefAndDoubleArguments()
    {
        using var loader = LoadSampleFromStream();

        loader.MoveToLineArgument(138, 6);
        Assert.Equal((uint)0, loader.GetOptionalRefArgument());

        loader.MoveToLineArgument(138, 9);
        var elevation = loader.GetOptionalDoubleParam(double.NaN);
        Assert.False(double.IsNaN(elevation));
        Assert.InRange(elevation, -1E-10, 1E-10);
    }

    [Fact]
    public void CanDecodeP21StringArgument()
    {
        using var loader = LoadSampleFromStream();

        loader.MoveToLineArgument(1, 1);
        var decoded = loader.GetDecodedStringArgument();

        Assert.Contains("Autodesk Revit 2021 (ENU)", decoded, StringComparison.Ordinal);
        Assert.EndsWith("Cé", decoded, StringComparison.Ordinal);
    }

    [Fact]
    public void CanNavigateExpressIds()
    {
        using var loader = LoadSampleFromStream();

        Assert.True(loader.IsValidExpressID(244));
        Assert.False(loader.IsValidExpressID(10));
        Assert.Equal((uint)11, loader.GetNextExpressID(9));

        loader.MoveToLineArgument(244, 0);
        Assert.Equal((uint)244, loader.GetCurrentLineExpressID());
    }

    [Fact]
    public void CanRoundtripSaveAndReload()
    {
        using var loader = LoadSampleFromStream();
        using var output = new MemoryStream();
        loader.SaveFile(output, orderLinesByExpressId: true);

        output.Position = 0;
        using var reloaded = CreateLoader();
        reloaded.LoadFile(output);

        Assert.Equal(loader.GetAllLines().Count, reloaded.GetAllLines().Count);
        Assert.Equal(loader.GetMaxExpressId(), reloaded.GetMaxExpressId());
        Assert.Equal(loader.GetLineType(244), reloaded.GetLineType(244));
    }

    [Fact]
    public void CanParseUsingMemoryMapOverload()
    {
        using var fromStream = LoadSampleFromStream();
        using var fromMap = CreateLoader();
        fromMap.LoadFile(SamplePath);

        Assert.Equal(fromStream.GetAllLines().Count, fromMap.GetAllLines().Count);
        Assert.Equal(fromStream.GetMaxExpressId(), fromMap.GetMaxExpressId());
        Assert.Equal(fromStream.GetSchema(), fromMap.GetSchema());
    }

    private static IfcLoader LoadSampleFromStream()
    {
        var loader = CreateLoader();
        using var stream = File.OpenRead(SamplePath);
        loader.LoadFile(stream);
        return loader;
    }

    private static IfcLoader CreateLoader()
    {
        return new IfcLoader(
            tapeSize: TapeSize,
            memoryLimit: MemoryLimit,
            lineWriterBuffer: LineWriterBuffer,
            schemaManager: SchemaManager
        );
    }

    private static string ResolveSampleIfcPath()
    {
        var candidates = new[]
        {
            Path.Combine(Directory.GetCurrentDirectory(), "tests", "ifcfiles", "public", "example.ifc"),
            Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "tests", "ifcfiles", "public", "example.ifc"),
            Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "tests", "ifcfiles", "public", "example.ifc")),
            "/Users/huyluong/Documents/engine_web-ifc/tests/ifcfiles/public/example.ifc"
        };

        foreach (var path in candidates)
        {
            var fullPath = Path.GetFullPath(path);
            if (File.Exists(fullPath))
            {
                return fullPath;
            }
        }

        throw new FileNotFoundException("Could not find tests/ifcfiles/public/example.ifc for specs.");
    }
}
