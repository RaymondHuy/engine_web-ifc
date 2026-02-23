using System.Diagnostics;
using System.Text.Json;

namespace BazanCDE.Parsing;

internal static class Program
{
    private const uint TapeSize = 64 * 1024 * 1024;
    private const ulong MemoryLimit = 2UL * 1024 * 1024 * 1024;
    private const uint LineWriterBuffer = 10_000;
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase
    };

    private static readonly string[] SupportedParityTypeNames =
    [
        "IFCEXTRUDEDAREASOLID",
        "IFCREVOLVEDAREASOLID",
        "IFCSURFACECURVESWEPTAREASOLID",
        "IFCFIXEDREFERENCESWEPTAREASOLID",
        "IFCSWEPTDISKSOLID",
        "IFCBSPLINESURFACE",
        "IFCBSPLINESURFACEWITHKNOTS",
        "IFCRATIONALBSPLINESURFACEWITHKNOTS",
        "IFCADVANCEDFACE",
        "IFCBOOLEANRESULT",
        "IFCBOOLEANCLIPPINGRESULT"
    ];

    private static int Main(string[] args)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help")
        {
            PrintUsage();
            return args.Length == 0 ? 1 : 0;
        }

        var unknownOptions = args
            .Where(a => a.StartsWith("--", StringComparison.Ordinal) && !IsKnownOption(a))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray();
        if (unknownOptions.Length > 0)
        {
            Console.Error.WriteLine($"Unknown option(s): {string.Join(", ", unknownOptions)}");
            PrintUsage();
            return 2;
        }

        var useMemoryMap = args.Any(a => string.Equals(a, "--mmap", StringComparison.OrdinalIgnoreCase));
        var parityJsonMode = args.Any(a => string.Equals(a, "--parity-json", StringComparison.OrdinalIgnoreCase));
        var positionalArgs = args
            .Where(a => !a.StartsWith("--", StringComparison.Ordinal))
            .ToArray();

        if (positionalArgs.Length == 0)
        {
            PrintUsage();
            return 1;
        }

        var inputPath = Path.GetFullPath(positionalArgs[0]);
        if (!File.Exists(inputPath))
        {
            Console.Error.WriteLine($"IFC file not found: {inputPath}");
            return 2;
        }

        if (parityJsonMode)
        {
            if (positionalArgs.Length > 1)
            {
                Console.Error.WriteLine("Unexpected positional arguments in --parity-json mode.");
                PrintUsage();
                return 2;
            }

            return RunParityJsonMode(inputPath, useMemoryMap);
        }

        var topTypes = 20;
        if (positionalArgs.Length > 1 && (!int.TryParse(positionalArgs[1], out topTypes) || topTypes <= 0))
        {
            Console.Error.WriteLine("Invalid topTypes argument. Expected a positive integer.");
            PrintUsage();
            return 2;
        }

        try
        {
            var schemaManager = new DefaultIfcSchemaManager();
            using var loader = new IfcLoader(
                tapeSize: TapeSize,
                memoryLimit: MemoryLimit,
                lineWriterBuffer: LineWriterBuffer,
                schemaManager: schemaManager
            );

            var stopwatch = Stopwatch.StartNew();
            if (useMemoryMap)
            {
                loader.LoadFile(inputPath);
            }
            else
            {
                using var stream = File.OpenRead(inputPath);
                loader.LoadFile(stream);
            }

            stopwatch.Stop();

            var schema = loader.GetSchema();
            var expressIds = loader.GetAllLines().OrderBy(x => x).ToArray();
            var maxExpressId = loader.GetMaxExpressId();

            Console.WriteLine($"File: {inputPath}");
            Console.WriteLine($"Schema: {schemaManager.GetSchemaName(schema)}");
            Console.WriteLine($"Data lines: {expressIds.Length}");
            Console.WriteLine($"Max express id: #{maxExpressId}");
            Console.WriteLine($"Parse time: {stopwatch.ElapsedMilliseconds} ms");
            Console.WriteLine($"Source mode: {(useMemoryMap ? "memory-map" : "stream")}");
            if (expressIds.Length > 0)
            {
                Console.WriteLine($"Express id range: #{expressIds[0]} .. #{expressIds[^1]}");
            }

            var typeCounts = new Dictionary<uint, int>();
            foreach (var expressId in expressIds)
            {
                var typeCode = loader.GetLineType(expressId);
                if (typeCode == 0)
                {
                    continue;
                }

                typeCounts[typeCode] = typeCounts.GetValueOrDefault(typeCode) + 1;
            }

            Console.WriteLine();
            Console.WriteLine($"Top {Math.Min(topTypes, typeCounts.Count)} entity types:");
            foreach (var pair in typeCounts
                         .OrderByDescending(x => x.Value)
                         .ThenBy(x => schemaManager.IfcTypeCodeToType(x.Key), StringComparer.Ordinal)
                         .Take(topTypes))
            {
                var typeName = schemaManager.IfcTypeCodeToType(pair.Key);
                if (string.IsNullOrEmpty(typeName))
                {
                    typeName = $"0x{pair.Key:X8}";
                }

                Console.WriteLine($"  {typeName,-36} {pair.Value,8}");
            }

            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"Failed to read IFC file: {ex.Message}");
            return 3;
        }
    }

    private static int RunParityJsonMode(string inputPath, bool useMemoryMap)
    {
        try
        {
            var schemaManager = new DefaultIfcSchemaManager();
            using var loader = new IfcLoader(
                tapeSize: TapeSize,
                memoryLimit: MemoryLimit,
                lineWriterBuffer: LineWriterBuffer,
                schemaManager: schemaManager
            );

            if (useMemoryMap)
            {
                loader.LoadFile(inputPath);
            }
            else
            {
                using var stream = File.OpenRead(inputPath);
                loader.LoadFile(stream);
            }

            var extractor = new IfcExtrudedGeometryExtractor(loader, schemaManager);
            var supportedTypeCodes = new HashSet<uint>(
                SupportedParityTypeNames
                    .Select(schemaManager.IfcTypeToTypeCode)
                    .Where(typeCode => typeCode != 0));

            var meshes = new List<ParityMeshMetric>();
            long triangleCount = 0;

            foreach (var expressId in loader.GetAllLines().OrderBy(x => x))
            {
                var lineType = loader.GetLineType(expressId);
                if (!supportedTypeCodes.Contains(lineType))
                {
                    continue;
                }

                if (!extractor.TryBuildMesh(expressId, out _, out var meshTriangleCount))
                {
                    continue;
                }

                if (meshTriangleCount <= 0)
                {
                    continue;
                }

                meshes.Add(new ParityMeshMetric(expressId, meshTriangleCount));
                triangleCount += meshTriangleCount;
            }

            var payload = new ParityPayload(
                MeshCount: meshes.Count,
                TriangleCount: triangleCount,
                Meshes: meshes);

            Console.WriteLine(JsonSerializer.Serialize(payload, JsonOptions));
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"Failed to generate parity JSON: {ex.Message}");
            return 4;
        }
    }

    private static bool IsKnownOption(string arg)
    {
        return string.Equals(arg, "--mmap", StringComparison.OrdinalIgnoreCase)
            || string.Equals(arg, "--parity-json", StringComparison.OrdinalIgnoreCase)
            || string.Equals(arg, "--help", StringComparison.OrdinalIgnoreCase);
    }

    private static void PrintUsage()
    {
        Console.WriteLine("Usage:");
        Console.WriteLine("  dotnet run --project csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- <path-to-ifc> [topTypes] [--mmap]");
        Console.WriteLine("  dotnet run --project csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- --parity-json <path-to-ifc> [--mmap]");
        Console.WriteLine();
        Console.WriteLine("Examples:");
        Console.WriteLine("  dotnet run --project csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- tests/ifcfiles/public/example.ifc");
        Console.WriteLine("  dotnet run --project csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- tests/ifcfiles/public/example.ifc 30");
        Console.WriteLine("  dotnet run --project csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- tests/ifcfiles/public/example.ifc 30 --mmap");
        Console.WriteLine("  dotnet run --project csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- --parity-json tests/ifcfiles/public/example.ifc");
    }

    private sealed record ParityPayload(int MeshCount, long TriangleCount, IReadOnlyList<ParityMeshMetric> Meshes);

    private sealed record ParityMeshMetric(uint ExpressId, int TriangleCount);
}
