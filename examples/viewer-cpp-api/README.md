# C++ API Viewer

This is a standalone frontend for the native Drogon API server. It does not
import `web-ifc-api` and does not require any `.wasm` files in the browser.

## Run

Start the C++ API server first:

```bash
./build/web-ifc-drogon/web-ifc-drogon-api 18080
```

Then serve the frontend:

```bash
npm run dev:viewer-cpp-api
```

Open:

```text
http://127.0.0.1:8081
```

The frontend sends IFC files to:

```text
POST http://127.0.0.1:18080/api/ifc/upload
GET  http://127.0.0.1:18080/api/ifc/geometry?filePath=<uploaded-path>
```

To point the page at a different API server, either edit the API base URL field
or pass it in the query string:

```text
http://127.0.0.1:8081?api=http://127.0.0.1:18081
```
