# BazanCDE Parser (C# Port)

This folder contains a C# port of the parser components from the C++ implementation:

- `IfcTokenType.cs`
- `IfcTokenStream.cs`
- `IfcLoader.cs`
- `IfcStringParsing.cs`
- `IfcGuidUtils.cs`
- `IfcSchema.cs` (schema contracts used by the loader)
- `DefaultIfcSchemaManager.cs` + `IfcTypeNameMap.cs` (default schema manager + generated type map)
- `IfcElementTypeNames.cs` (IFC element list mirrored from C++ schema manager)
- `LoaderSettings.cs` + `ModelManager.cs` (model lifecycle manager layer)
- `Geometry/IfcGeometryProcessor.cs` + `Geometry/IfcGeometryLoader.cs` + `Geometry/IfcExtrudedGeometryExtractor.cs` (geometry processing layer for swept/extruded/revolved solids, boolean results, curve extraction including polyline/composite/circle/bspline, bspline surface tessellation, and advanced-face trim over bspline surfaces)
- `Geometry/IfcGeometryModels.cs` (flat/composed mesh DTOs and parsed mesh models)
- `Geometry/Operations/*` (ported operations layer: curve/profile primitives, extrusion/sweep/revolution, AABB and boolean engine with BVH/ray-triangle inside-outside classification, relevant-face normalization, and shared-plane contour triangulation)

## Notes

- The port keeps the same token/tape model (`IfcTokenStream` + `IfcLoader`) so line indexing and random access by tape offset work similarly.
- `IfcStringParsing` includes STEP P21 encode/decode helpers used by the parser bridge.
- `DefaultIfcSchemaManager` is included and can be used directly:

```csharp
var schemaManager = new DefaultIfcSchemaManager();
var loader = new IfcLoader(
    tapeSize: 64 * 1024 * 1024,
    memoryLimit: 2UL * 1024 * 1024 * 1024,
    lineWriterBuffer: 10_000,
    schemaManager: schemaManager
);
```

## Build

```bash
dotnet build /Users/huyluong/Documents/engine_web-ifc/csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj
```

## Run (CLI / Rider)

- Project is configured as runnable (`OutputType=Exe`).
- Default launch profile is in:
  - `/Users/huyluong/Documents/engine_web-ifc/csharp/BazanCDE.Parsing/Properties/launchSettings.json`
  - Profile names:
    - `Read IFC (example)` (stream)
    - `Read IFC (memory map)` (memory mapped)
  - Default args: `../../tests/ifcfiles/public/example.ifc 20`

Run from terminal:

```bash
dotnet run --project /Users/huyluong/Documents/engine_web-ifc/csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- ../../tests/ifcfiles/public/example.ifc 20
dotnet run --project /Users/huyluong/Documents/engine_web-ifc/csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj -- ../../tests/ifcfiles/public/example.ifc 20 --mmap
```

## Load API

Both loading styles are supported:

```csharp
// Existing stream mode
using var stream = File.OpenRead("model.ifc");
loader.LoadFile(stream);

// New memory-mapped overload
loader.LoadFile("model.ifc");
```

## Model Manager API

```csharp
var modelManager = new ModelManager(mtEnabled: false);
var modelId = modelManager.CreateModel(new LoaderSettings());

var loader = modelManager.GetIfcLoader(modelId)!;
using var stream = File.OpenRead("model.ifc");
loader.LoadFile(stream);

var processor = modelManager.GetGeometryProcessor(modelId)!;
var parsed = processor.AnalyzeModel(maxMeshes: 1000, maxTriangles: 500_000);
```

## Operations API

```csharp
using BazanCDE.Parsing.Operations;

var extrusion = new Extrusion();
extrusion.SetValues(
    profile: [-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0],
    dir: [0, 0, 1],
    len: 2.0,
    cuttingPlaneNormal: [0, 0, 1],
    cuttingPlanePos: [0, 0, 0],
    cap: true);
Buffers mesh = extrusion.GetBuffers();
```

## Regenerate Type Map

When `src/ts/ifc-schema.ts` changes, regenerate `IfcTypeNameMap.cs`:

```bash
/Users/huyluong/Documents/engine_web-ifc/csharp/BazanCDE.Parsing/scripts/generate_type_map.sh
```
