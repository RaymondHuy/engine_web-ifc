using System.Collections.Concurrent;
using Microsoft.AspNetCore.Mvc;
using BazanCDE.Parsing;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddCors(options =>
{
    options.AddDefaultPolicy(policy =>
    {
        policy.AllowAnyOrigin();
        policy.AllowAnyMethod();
        policy.AllowAnyHeader();
    });
});
builder.Services.AddSingleton<IfcModelStore>();

var app = builder.Build();

app.UseCors();

var viewerRoot = ResolveViewerRoot(app.Environment.ContentRootPath);
var sampleIfcPath = ResolveSampleIfcPath(app.Environment.ContentRootPath);

app.MapGet("/", () =>
{
    if (viewerRoot is null)
    {
        return Results.Problem("Viewer assets not found.");
    }

    return Results.File(Path.Combine(viewerRoot, "index.html"), "text/html; charset=utf-8");
});

app.MapGet("/app.js", () =>
{
    if (viewerRoot is null)
    {
        return Results.NotFound(new { error = "Viewer assets not found." });
    }

    return Results.File(Path.Combine(viewerRoot, "app.js"), "application/javascript; charset=utf-8");
});

app.MapGet("/styles.css", () =>
{
    if (viewerRoot is null)
    {
        return Results.NotFound(new { error = "Viewer assets not found." });
    }

    return Results.File(Path.Combine(viewerRoot, "styles.css"), "text/css; charset=utf-8");
});

app.MapGet("/api/health", () => Results.Ok(new { status = "ok" }));
app.MapGet("/api", () => Results.Ok(new
{
    name = "BazanCDE.ViewerApi",
    endpoints = new[]
    {
        "GET /api/health",
        "GET /api/models/sample",
        "GET /api/models/sample/summary",
        "POST /api/models/upload?fileName=example.ifc (body: application/octet-stream)",
        "GET /api/models/{modelId}/ifc",
        "GET /api/models/{modelId}/summary",
        "DELETE /api/models/{modelId}"
    }
}));

app.MapGet("/api/models/sample", async () =>
{
    if (sampleIfcPath is null)
    {
        return Results.NotFound(new { error = "Sample IFC not found." });
    }

    var bytes = await File.ReadAllBytesAsync(sampleIfcPath);
    return Results.File(bytes, "application/octet-stream", "example.ifc");
});

app.MapGet("/api/models/sample/summary", async () =>
{
    if (sampleIfcPath is null)
    {
        return Results.NotFound(new { error = "Sample IFC not found." });
    }

    await using var stream = File.OpenRead(sampleIfcPath);
    var summary = IfcModelAnalyzer.Analyze(stream);
    return Results.Ok(summary);
});

app.MapPost("/api/models/upload", async (HttpRequest request, [FromQuery] string? fileName, IfcModelStore modelStore) =>
{
    if (request.ContentLength is null || request.ContentLength == 0)
    {
        return Results.BadRequest(new { error = "Empty upload." });
    }

    // Demo guardrail to avoid accidental huge in-memory uploads.
    const long maxUploadSize = 200L * 1024 * 1024;
    if (request.ContentLength > maxUploadSize)
    {
        return Results.BadRequest(new { error = $"File is larger than {maxUploadSize} bytes." });
    }

    byte[] bytes;
    await using (var memory = new MemoryStream())
    {
        await request.Body.CopyToAsync(memory);
        bytes = memory.ToArray();
    }

    var safeFileName = string.IsNullOrWhiteSpace(fileName) ? "upload.ifc" : fileName;
    var summary = IfcModelAnalyzer.Analyze(bytes);
    var storedModel = modelStore.Add(safeFileName, bytes, summary);
    return Results.Ok(new
    {
        modelId = storedModel.ModelId,
        fileName = storedModel.FileName,
        sizeBytes = storedModel.Bytes.LongLength,
        summary = storedModel.Summary
    });
});

app.MapGet("/api/models/{modelId}/ifc", (string modelId, IfcModelStore modelStore) =>
{
    if (!modelStore.TryGet(modelId, out var model))
    {
        return Results.NotFound(new { error = "Model not found." });
    }

    return Results.File(model.Bytes, "application/octet-stream", model.FileName);
});

app.MapGet("/api/models/{modelId}/summary", (string modelId, IfcModelStore modelStore) =>
{
    if (!modelStore.TryGet(modelId, out var model))
    {
        return Results.NotFound(new { error = "Model not found." });
    }

    return Results.Ok(model.Summary);
});

app.MapDelete("/api/models/{modelId}", (string modelId, IfcModelStore modelStore) =>
{
    return modelStore.Remove(modelId)
        ? Results.NoContent()
        : Results.NotFound(new { error = "Model not found." });
});

app.Run();

static string? ResolveViewerRoot(string contentRootPath)
{
    var candidates = new[]
    {
        Path.Combine(AppContext.BaseDirectory, "wwwroot"),
        Path.Combine(contentRootPath, "wwwroot"),
        Path.Combine(contentRootPath, "csharp", "BazanCDE.ViewerApi", "wwwroot"),
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "wwwroot")),
    };

    return candidates.FirstOrDefault(Directory.Exists);
}

static string? ResolveSampleIfcPath(string contentRootPath)
{
    var candidates = new[]
    {
        Path.Combine(contentRootPath, "examples", "example.ifc"),
        Path.Combine(contentRootPath, "..", "..", "examples", "example.ifc"),
        Path.Combine(contentRootPath, "csharp", "BazanCDE.ViewerApi", "..", "..", "examples", "example.ifc"),
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "..", "examples", "example.ifc")),
    };

    return candidates.Select(Path.GetFullPath).FirstOrDefault(File.Exists);
}

internal sealed class IfcModelAnalyzer
{
    private const uint TapeSize = 64 * 1024 * 1024;
    private const ulong MemoryLimit = 2UL * 1024 * 1024 * 1024;
    private const uint LineWriterBuffer = 10_000;

    public static IfcModelSummary Analyze(byte[] modelBytes, int topTypes = 20)
    {
        using var stream = new MemoryStream(modelBytes, writable: false);
        return Analyze(stream, topTypes);
    }

    public static IfcModelSummary Analyze(Stream modelStream, int topTypes = 20)
    {
        modelStream.Position = 0;

        var schemaManager = new DefaultIfcSchemaManager();
        using var loader = new IfcLoader(
            tapeSize: TapeSize,
            memoryLimit: MemoryLimit,
            lineWriterBuffer: LineWriterBuffer,
            schemaManager: schemaManager
        );

        loader.LoadFile(modelStream);

        var expressIds = loader.GetAllLines();
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

        var top = typeCounts
            .Select(x =>
            {
                var typeName = schemaManager.IfcTypeCodeToType(x.Key);
                if (string.IsNullOrWhiteSpace(typeName))
                {
                    typeName = $"0x{x.Key:X8}";
                }

                return new IfcTypeCount(typeName, x.Key, x.Value);
            })
            .OrderByDescending(x => x.Count)
            .ThenBy(x => x.TypeName, StringComparer.Ordinal)
            .Take(Math.Max(1, topTypes))
            .ToArray();

        return new IfcModelSummary(
            Schema: schemaManager.GetSchemaName(loader.GetSchema()),
            DataLineCount: expressIds.Count,
            MaxExpressId: loader.GetMaxExpressId(),
            TopTypes: top
        );
    }
}

internal sealed class IfcModelStore
{
    private readonly ConcurrentDictionary<string, StoredIfcModel> _models = new();

    public StoredIfcModel Add(string fileName, byte[] bytes, IfcModelSummary summary)
    {
        var id = Guid.NewGuid().ToString("N");
        var model = new StoredIfcModel(id, fileName, bytes, summary, DateTimeOffset.UtcNow);
        _models[id] = model;
        return model;
    }

    public bool TryGet(string modelId, out StoredIfcModel model)
    {
        return _models.TryGetValue(modelId, out model!);
    }

    public bool Remove(string modelId)
    {
        return _models.TryRemove(modelId, out _);
    }
}

internal sealed record IfcModelSummary(
    string Schema,
    int DataLineCount,
    uint MaxExpressId,
    IReadOnlyList<IfcTypeCount> TopTypes
);

internal sealed record IfcTypeCount(string TypeName, uint TypeCode, int Count);

internal sealed record StoredIfcModel(
    string ModelId,
    string FileName,
    byte[] Bytes,
    IfcModelSummary Summary,
    DateTimeOffset CreatedAt
);
