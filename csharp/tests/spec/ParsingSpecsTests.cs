using BazanCDE.Parsing;
using BazanCDE.Parsing.Operations;
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

    [Fact]
    public void SchemaManagerCanDetectIfcElements()
    {
        var wallType = SchemaManager.IfcTypeToTypeCode("IFCWALL");
        var ownerHistoryType = SchemaManager.IfcTypeToTypeCode("IFCOWNERHISTORY");

        Assert.True(SchemaManager.IsIfcElement(wallType));
        Assert.False(SchemaManager.IsIfcElement(ownerHistoryType));
        Assert.Contains(wallType, SchemaManager.GetIfcElementList());
    }

    [Fact]
    public void ModelManagerCanCreateOpenAndCloseModel()
    {
        using var modelManager = new ModelManager(mtEnabled: false, SchemaManager);
        var modelId = modelManager.CreateModel(new LoaderSettings());

        Assert.True(modelManager.IsModelOpen(modelId));

        var loader = modelManager.GetIfcLoader(modelId);
        Assert.NotNull(loader);

        using var stream = File.OpenRead(SamplePath);
        loader!.LoadFile(stream);

        var processor = modelManager.GetGeometryProcessor(modelId);
        Assert.NotNull(processor);
        Assert.True(processor!.AnalyzeModel(maxMeshes: 1, maxTriangles: 100).MeshCount >= 0);

        modelManager.CloseModel(modelId);
        Assert.False(modelManager.IsModelOpen(modelId));
    }

    [Fact]
    public void OperationsExtrusionProducesTriangles()
    {
        var extrusion = new Extrusion();
        extrusion.SetValues(
            profile:
            [
                -0.5, -0.5, 0,
                0.5, -0.5, 0,
                0.5, 0.5, 0,
                -0.5, 0.5, 0
            ],
            dir: [0, 0, 1],
            len: 2.0,
            cuttingPlaneNormal: [0, 0, 1],
            cuttingPlanePos: [0, 0, 0],
            cap: true);

        var buffers = extrusion.GetBuffers();
        Assert.NotEmpty(buffers.VertexData);
        Assert.NotEmpty(buffers.IndexData);
        Assert.True(buffers.IndexData.Count % 3 == 0);
    }

    [Fact]
    public void OperationsRevolutionProducesTriangles()
    {
        var revolution = new Revolve();
        revolution.SetValues(
            profile:
            [
                1.0, 0.0, 0.0,
                1.0, 0.0, 2.0
            ],
            transform:
            [
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
            ],
            startDegrees: 0,
            endDegrees: 360,
            numRots: 24);

        var buffers = revolution.GetBuffers();
        Assert.NotEmpty(buffers.VertexData);
        Assert.NotEmpty(buffers.IndexData);
    }

    [Fact]
    public void OperationsBooleanUnionProducesMesh()
    {
        var boolean = new BazanCDE.Parsing.Operations.Boolean();
        boolean.SetValues(
            triangles:
            [
                // tri 1
                0, 0, 0,
                1, 0, 0,
                0, 1, 0,
                // tri 2
                1, 0, 0,
                1, 1, 0,
                0, 1, 0
            ],
            op: "UNION");

        boolean.SetSecond(
            triangles:
            [
                // shifted quad as 2 tris
                0, 0, 1,
                1, 0, 1,
                0, 1, 1,
                1, 0, 1,
                1, 1, 1,
                0, 1, 1
            ]);

        var buffers = boolean.GetBuffers();
        Assert.NotEmpty(buffers.VertexData);
        Assert.NotEmpty(buffers.IndexData);
    }

    [Fact]
    public void OperationsBooleanDifferenceProducesMeshOnOverlappingCubes()
    {
        var boolean = new BazanCDE.Parsing.Operations.Boolean();
        boolean.SetValues(CreateBoxTriangles(0, 0, 0, 1, 1, 1), "DIFFERENCE");
        boolean.SetSecond(CreateBoxTriangles(0.4, 0.4, 0.4, 1, 1, 1));

        var buffers = boolean.GetBuffers();
        Assert.NotEmpty(buffers.VertexData);
        Assert.NotEmpty(buffers.IndexData);
        Assert.True(buffers.IndexData.Count % 3 == 0);
    }

    [Fact]
    public void AnalyzerSupportsRevolvedAndBooleanResultFromSyntheticIfc()
    {
        var ifc = """
                  ISO-10303-21;
                  HEADER;
                  FILE_DESCRIPTION(('ViewDefinition [CoordinationView]'),'2;1');
                  FILE_NAME('synthetic.ifc','2026-02-22T00:00:00',('tester'),('org'),'preproc','originating','auth');
                  FILE_SCHEMA(('IFC2X3'));
                  ENDSEC;
                  DATA;
                  #1=IFCCARTESIANPOINT((0.,0.,0.));
                  #2=IFCDIRECTION((0.,0.,1.));
                  #3=IFCDIRECTION((1.,0.,0.));
                  #4=IFCAXIS2PLACEMENT3D(#1,#2,#3);
                  #5=IFCCARTESIANPOINT((0.,0.,0.));
                  #6=IFCDIRECTION((0.,0.,1.));
                  #7=IFCAXIS1PLACEMENT(#5,#6);
                  #8=IFCCARTESIANPOINT((0.,0.));
                  #9=IFCDIRECTION((1.,0.));
                  #10=IFCAXIS2PLACEMENT2D(#8,#9);
                  #11=IFCRECTANGLEPROFILEDEF(.AREA.,$,#10,2.,1.);
                  #12=IFCREVOLVEDAREASOLID(#11,#4,#7,6.283185307179586);
                  #13=IFCEXTRUDEDAREASOLID(#11,#4,#2,1.);
                  #14=IFCEXTRUDEDAREASOLID(#11,#4,#2,0.5);
                  #15=IFCBOOLEANRESULT(.UNION.,#13,#14);
                  ENDSEC;
                  END-ISO-10303-21;
                  """;

        var bytes = System.Text.Encoding.UTF8.GetBytes(ifc);
        var parsed = IfcModelGeometryAnalyzer.Analyze(bytes, maxMeshes: 50, maxTriangles: 200_000);

        Assert.True(parsed.MeshCount >= 2);
        Assert.True(parsed.TriangleCount > 0);
        Assert.Contains(parsed.Meshes, m => m.ExpressId == 12);
        Assert.Contains(parsed.Meshes, m => m.ExpressId == 13);
        Assert.Contains(parsed.Meshes, m => m.ExpressId == 15);
    }

    [Fact]
    public void AnalyzerSupportsBsplineDirectrixForSweptDisk()
    {
        var ifc = """
                  ISO-10303-21;
                  HEADER;
                  FILE_DESCRIPTION(('ViewDefinition [CoordinationView]'),'2;1');
                  FILE_NAME('synthetic-bspline.ifc','2026-02-23T00:00:00',('tester'),('org'),'preproc','originating','auth');
                  FILE_SCHEMA(('IFC2X3'));
                  ENDSEC;
                  DATA;
                  #1=IFCCARTESIANPOINT((0.,0.,0.));
                  #2=IFCCARTESIANPOINT((1.,0.,0.));
                  #3=IFCCARTESIANPOINT((2.,1.,0.));
                  #4=IFCCARTESIANPOINT((3.,1.,0.));
                  #10=IFCBSPLINECURVE(3,(#1,#2,#3,#4),.UNSPECIFIED.,.F.,.F.);
                  #11=IFCSWEPTDISKSOLID(#10,0.08,$,$);
                  ENDSEC;
                  END-ISO-10303-21;
                  """;

        var parsed = IfcModelGeometryAnalyzer.Analyze(System.Text.Encoding.UTF8.GetBytes(ifc), maxMeshes: 50, maxTriangles: 200_000);
        Assert.Contains(parsed.Meshes, m => m.ExpressId == 11);
        Assert.True(parsed.TriangleCount > 0);
    }

    [Fact]
    public void AnalyzerSupportsArbitraryProfileWithVoidsExtrusion()
    {
        var ifc = """
                  ISO-10303-21;
                  HEADER;
                  FILE_DESCRIPTION(('ViewDefinition [CoordinationView]'),'2;1');
                  FILE_NAME('synthetic-voids.ifc','2026-02-23T00:00:00',('tester'),('org'),'preproc','originating','auth');
                  FILE_SCHEMA(('IFC2X3'));
                  ENDSEC;
                  DATA;
                  #1=IFCCARTESIANPOINT((0.,0.));
                  #2=IFCCARTESIANPOINT((4.,0.));
                  #3=IFCCARTESIANPOINT((4.,4.));
                  #4=IFCCARTESIANPOINT((0.,4.));
                  #5=IFCCARTESIANPOINT((1.,1.));
                  #6=IFCCARTESIANPOINT((3.,1.));
                  #7=IFCCARTESIANPOINT((3.,3.));
                  #8=IFCCARTESIANPOINT((1.,3.));
                  #9=IFCPOLYLINE((#1,#2,#3,#4,#1));
                  #10=IFCPOLYLINE((#5,#6,#7,#8,#5));
                  #11=IFCARBITRARYPROFILEDEFWITHVOIDS(.AREA.,$,#9,(#10));
                  #20=IFCCARTESIANPOINT((0.,0.,0.));
                  #21=IFCDIRECTION((0.,0.,1.));
                  #22=IFCDIRECTION((1.,0.,0.));
                  #23=IFCAXIS2PLACEMENT3D(#20,#21,#22);
                  #24=IFCEXTRUDEDAREASOLID(#11,#23,#21,2.);
                  ENDSEC;
                  END-ISO-10303-21;
                  """;

        var parsed = IfcModelGeometryAnalyzer.Analyze(System.Text.Encoding.UTF8.GetBytes(ifc), maxMeshes: 20, maxTriangles: 300_000);
        Assert.Contains(parsed.Meshes, m => m.ExpressId == 24);
        Assert.True(parsed.TriangleCount > 0);
    }

    [Fact]
    public void AnalyzerSupportsBsplineSurfaceWithKnots()
    {
        var ifc = """
                  ISO-10303-21;
                  HEADER;
                  FILE_DESCRIPTION(('ViewDefinition [CoordinationView]'),'2;1');
                  FILE_NAME('synthetic-bspline-surface.ifc','2026-02-23T00:00:00',('tester'),('org'),'preproc','originating','auth');
                  FILE_SCHEMA(('IFC2X3'));
                  ENDSEC;
                  DATA;
                  #1=IFCCARTESIANPOINT((0.,0.,0.));
                  #2=IFCCARTESIANPOINT((1.,0.,0.2));
                  #3=IFCCARTESIANPOINT((0.,1.,0.1));
                  #4=IFCCARTESIANPOINT((1.,1.,0.3));
                  #10=IFCBSPLINESURFACEWITHKNOTS(
                      1,
                      1,
                      ((#1,#2),(#3,#4)),
                      .UNSPECIFIED.,
                      .F.,
                      .F.,
                      .F.,
                      (2,2),
                      (2,2),
                      (0.,1.),
                      (0.,1.),
                      .UNSPECIFIED.);
                  ENDSEC;
                  END-ISO-10303-21;
                  """;

        var parsed = IfcModelGeometryAnalyzer.Analyze(System.Text.Encoding.UTF8.GetBytes(ifc), maxMeshes: 20, maxTriangles: 300_000);
        Assert.Contains(parsed.Meshes, m => m.ExpressId == 10);
        Assert.True(parsed.TriangleCount > 0);
    }

    [Fact]
    public void AnalyzerSupportsAdvancedFaceBsplineTrim()
    {
        var ifc = """
                  ISO-10303-21;
                  HEADER;
                  FILE_DESCRIPTION(('ViewDefinition [CoordinationView]'),'2;1');
                  FILE_NAME('synthetic-advancedface.ifc','2026-02-23T00:00:00',('tester'),('org'),'preproc','originating','auth');
                  FILE_SCHEMA(('IFC2X3'));
                  ENDSEC;
                  DATA;
                  #1=IFCCARTESIANPOINT((0.,0.,0.));
                  #2=IFCCARTESIANPOINT((1.,0.,0.2));
                  #3=IFCCARTESIANPOINT((0.,1.,0.1));
                  #4=IFCCARTESIANPOINT((1.,1.,0.3));
                  #10=IFCBSPLINESURFACEWITHKNOTS(
                      1,
                      1,
                      ((#1,#2),(#3,#4)),
                      .UNSPECIFIED.,
                      .F.,
                      .F.,
                      .F.,
                      (2,2),
                      (2,2),
                      (0.,1.),
                      (0.,1.),
                      .UNSPECIFIED.);
                  #20=IFCPOLYLOOP((#1,#2,#4,#3));
                  #21=IFCFACEOUTERBOUND(#20,.T.);
                  #22=IFCADVANCEDFACE((#21),#10,.T.);
                  ENDSEC;
                  END-ISO-10303-21;
                  """;

        var parsed = IfcModelGeometryAnalyzer.Analyze(System.Text.Encoding.UTF8.GetBytes(ifc), maxMeshes: 20, maxTriangles: 300_000);
        Assert.Contains(parsed.Meshes, m => m.ExpressId == 22);
        Assert.True(parsed.TriangleCount > 0);
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

    private static double[] CreateBoxTriangles(double x, double y, double z, double sx, double sy, double sz)
    {
        var x0 = x;
        var y0 = y;
        var z0 = z;
        var x1 = x + sx;
        var y1 = y + sy;
        var z1 = z + sz;

        return
        [
            // Bottom
            x0, y0, z0, x1, y0, z0, x1, y1, z0,
            x0, y0, z0, x1, y1, z0, x0, y1, z0,
            // Top
            x0, y0, z1, x1, y1, z1, x1, y0, z1,
            x0, y0, z1, x0, y1, z1, x1, y1, z1,
            // Front
            x0, y0, z0, x1, y0, z1, x1, y0, z0,
            x0, y0, z0, x0, y0, z1, x1, y0, z1,
            // Back
            x0, y1, z0, x1, y1, z0, x1, y1, z1,
            x0, y1, z0, x1, y1, z1, x0, y1, z1,
            // Left
            x0, y0, z0, x0, y1, z0, x0, y1, z1,
            x0, y0, z0, x0, y1, z1, x0, y0, z1,
            // Right
            x1, y0, z0, x1, y1, z1, x1, y1, z0,
            x1, y0, z0, x1, y0, z1, x1, y1, z1
        ];
    }
}
