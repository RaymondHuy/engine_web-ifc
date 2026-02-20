using System.Diagnostics;

namespace WebIfc.Parsing;

internal static class Program
{
    private const uint TapeSize = 64 * 1024 * 1024;
    private const ulong MemoryLimit = 2UL * 1024 * 1024 * 1024;
    private const uint LineWriterBuffer = 10_000;

    private static int Main(string[] args)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help")
        {
            PrintUsage();
            return args.Length == 0 ? 1 : 0;
        }

        var inputPath = Path.GetFullPath(args[0]);
        if (!File.Exists(inputPath))
        {
            Console.Error.WriteLine($"IFC file not found: {inputPath}");
            return 2;
        }

        var topTypes = 20;
        if (args.Length > 1 && (!int.TryParse(args[1], out topTypes) || topTypes <= 0))
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

            using var stream = File.OpenRead(inputPath);
            var stopwatch = Stopwatch.StartNew();
            loader.LoadFile(stream);
            stopwatch.Stop();

            var schema = loader.GetSchema();
            var expressIds = loader.GetAllLines().OrderBy(x => x).ToArray();
            var maxExpressId = loader.GetMaxExpressId();

            Console.WriteLine($"File: {inputPath}");
            Console.WriteLine($"Schema: {schemaManager.GetSchemaName(schema)}");
            Console.WriteLine($"Data lines: {expressIds.Length}");
            Console.WriteLine($"Max express id: #{maxExpressId}");
            Console.WriteLine($"Parse time: {stopwatch.ElapsedMilliseconds} ms");
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

    private static void PrintUsage()
    {
        Console.WriteLine("Usage:");
        Console.WriteLine("  dotnet run --project csharp/WebIfc.Parsing/WebIfc.Parsing.csproj -p:OutputType=Exe -- <path-to-ifc> [topTypes]");
        Console.WriteLine();
        Console.WriteLine("Examples:");
        Console.WriteLine("  dotnet run --project csharp/WebIfc.Parsing/WebIfc.Parsing.csproj -p:OutputType=Exe -- tests/ifcfiles/public/example.ifc");
        Console.WriteLine("  dotnet run --project csharp/WebIfc.Parsing/WebIfc.Parsing.csproj -p:OutputType=Exe -- tests/ifcfiles/public/example.ifc 30");
    }
}
