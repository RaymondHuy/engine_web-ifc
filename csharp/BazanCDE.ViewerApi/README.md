# BazanCDE Viewer API (C#)

Second viewer that fetches IFC models via a C# backend API.

## What it includes

- ASP.NET Core API for:
  - `GET /api/models/sample` (sample IFC bytes)
  - `GET /api/models/sample/summary` (summary parsed by C# parser)
  - `POST /api/models/upload?fileName=...` (raw IFC body upload, store in memory, return model id + summary)
  - `GET /api/models/{modelId}/ifc`
  - `GET /api/models/{modelId}/summary`
  - `DELETE /api/models/{modelId}`
- Viewer UI at `/`:
  - load sample from API
  - upload IFC to API and render returned model
  - show schema/count/top entity types from C# summary

## Run

```bash
dotnet run --project /Users/huyluong/Documents/engine_web-ifc/csharp/BazanCDE.ViewerApi/BazanCDE.ViewerApi.csproj
```

Open:

- `http://localhost:5140/`

## Notes

- The browser viewer loads `web-ifc` and `three` from CDN (`unpkg`).
- Model parsing for summaries uses `/Users/huyluong/Documents/engine_web-ifc/csharp/BazanCDE.Parsing`.
