# C++ API Server

This folder contains the native Drogon HTTP API server implemented in
`web_ifc_drogon_main.cpp`.

## Requirements

- CMake 3.18+
- A C++20 compiler
- Network access on the first build, because CMake downloads Drogon and other
  native dependencies through `FetchContent`.

On macOS, install CMake with Homebrew if it is missing:

```bash
brew install cmake
```

## Build

Run these commands from the repository root:

```bash
cmake -S src/cpp -B build/web-ifc-drogon \
  -DCMAKE_BUILD_TYPE=Release \
  -DWEB_IFC_ENABLE_DROGON_API=ON \
  -DWEB_IFC_API_DEFAULT_PORT=18080

cmake --build build/web-ifc-drogon \
  --target web-ifc-drogon-api \
  --config Release \
  -j $(sysctl -n hw.ncpu)
```

On Linux, replace `$(sysctl -n hw.ncpu)` with `$(nproc)`.

## CLion

Open the repository root in CLion, not only the single `.cpp` file. CLion will
detect the root `CMakeLists.txt` and the `cpp-api-debug` preset.

If you already opened `src/cpp/api`, that works too. This folder has a small
wrapper `CMakeLists.txt` that forwards CLion to the real C++ project.

Use this run configuration:

```text
Target: web-ifc-drogon-api
Executable: web-ifc-drogon-api
Program arguments: 18080
Working directory: repository root
```

If CLion still shows red include errors after opening the project, reload CMake
from **Tools > CMake > Reload CMake Project** and wait for the first
`FetchContent` download to finish.

## Run

```bash
./build/web-ifc-drogon/web-ifc-drogon-api 18080
```

The server listens on:

```text
http://127.0.0.1:18080
```

You can choose another port by passing it as the first argument:

```bash
./build/web-ifc-drogon/web-ifc-drogon-api 18081
```

The API accepts upload bodies up to `512MB` by default. Override this with:

```bash
WEB_IFC_API_MAX_BODY_BYTES=1073741824 \
  ./build/web-ifc-drogon/web-ifc-drogon-api 18080
```

## Test

Health check:

```bash
curl http://127.0.0.1:18080/health
```

Upload an IFC file:

```bash
curl -F "file=@examples/example.ifc" \
  http://127.0.0.1:18080/api/ifc/upload
```

The upload endpoint stores files in `./uploads` relative to the directory where
the server process was started.

Get renderable geometry for a server-side IFC file:

```bash
curl "http://127.0.0.1:18080/api/ifc/geometry?filePath=examples/example.ifc"
```

## Frontend

The standalone frontend that calls this API is in `examples/viewer-cpp-api`.
It does not load the browser WASM package.

```bash
npm run dev:viewer-cpp-api
```

Open:

```text
http://127.0.0.1:8081
```

## Stop The Server

If the port is already in use:

```text
Address already in use (errno=48), Bind address failed at 0.0.0.0:18080
```

Find the process:

```bash
lsof -nP -iTCP:18080 -sTCP:LISTEN
```

Stop it by PID:

```bash
kill <PID>
```

Or stop whatever is listening on port `18080`:

```bash
kill $(lsof -tiTCP:18080 -sTCP:LISTEN)
```

If it does not stop:

```bash
kill -9 $(lsof -tiTCP:18080 -sTCP:LISTEN)
```

## Endpoints

```text
GET  /health
POST /api/ifc/upload
GET  /api/ifc/geometry?filePath=<path>
POST /api/ifc/geometry
GET  /api/ifc/summary?filePath=<path>&top=<number>
POST /api/ifc/summary
```

Note: `/health`, `/api/ifc/upload`, and `/api/ifc/geometry` were verified
locally. The `/api/ifc/summary` endpoint may need a parser stream fix before it
is safe to use with real IFC files.
