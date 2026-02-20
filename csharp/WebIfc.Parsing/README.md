# WebIfc Parser (C# Port)

This folder contains a C# port of the parser components from the C++ implementation:

- `IfcTokenType.cs`
- `IfcTokenStream.cs`
- `IfcLoader.cs`
- `IfcStringParsing.cs`
- `IfcGuidUtils.cs`
- `IfcSchema.cs` (schema contracts used by the loader)
- `DefaultIfcSchemaManager.cs` + `IfcTypeNameMap.cs` (default schema manager + generated type map)

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
dotnet build /Users/huyluong/Documents/engine_web-ifc/csharp/WebIfc.Parsing/WebIfc.Parsing.csproj
```

## Run (CLI / Rider)

- Project is configured as runnable (`OutputType=Exe`).
- Default launch profile is in:
  - `/Users/huyluong/Documents/engine_web-ifc/csharp/WebIfc.Parsing/Properties/launchSettings.json`
  - Profile names:
    - `Read IFC (example)` (stream)
    - `Read IFC (memory map)` (memory mapped)
  - Default args: `../../tests/ifcfiles/public/example.ifc 20`

Run from terminal:

```bash
dotnet run --project /Users/huyluong/Documents/engine_web-ifc/csharp/WebIfc.Parsing/WebIfc.Parsing.csproj -- ../../tests/ifcfiles/public/example.ifc 20
dotnet run --project /Users/huyluong/Documents/engine_web-ifc/csharp/WebIfc.Parsing/WebIfc.Parsing.csproj -- ../../tests/ifcfiles/public/example.ifc 20 --mmap
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

## Regenerate Type Map

When `src/ts/ifc-schema.ts` changes, regenerate `IfcTypeNameMap.cs`:

```bash
/Users/huyluong/Documents/engine_web-ifc/csharp/WebIfc.Parsing/scripts/generate_type_map.sh
```
