#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: scripts/compare-parity.sh <path-to-ifc>" >&2
  exit 1
fi

INPUT_PATH="$1"
INPUT_DIR="$(cd "$(dirname "$INPUT_PATH")" && pwd)"
INPUT_FILE="$(basename "$INPUT_PATH")"
IFC_PATH="$INPUT_DIR/$INPUT_FILE"

if [[ ! -f "$IFC_PATH" ]]; then
  echo "IFC file not found: $IFC_PATH" >&2
  exit 2
fi

if command -v cmake >/dev/null 2>&1; then
  CMAKE_BIN="$(command -v cmake)"
elif [[ -x "$HOME/Library/Python/3.9/bin/cmake" ]]; then
  CMAKE_BIN="$HOME/Library/Python/3.9/bin/cmake"
else
  echo "cmake not found. Install cmake or set CMAKE_BIN." >&2
  exit 3
fi

CPP_BUILD_DIR="${CPP_BUILD_DIR:-$ROOT_DIR/build/cpp-native}"
DOTNET_CMD="${DOTNET_CMD:-dotnet}"

CPP_JSON="$(mktemp "${TMPDIR:-/tmp}/cpp-parity.XXXXXX.json")"
CS_JSON="$(mktemp "${TMPDIR:-/tmp}/cs-parity.XXXXXX.json")"
trap 'rm -f "$CPP_JSON" "$CS_JSON"' EXIT

echo "[1/4] Configuring C++ build: $CPP_BUILD_DIR"
"$CMAKE_BIN" -S "$ROOT_DIR/src/cpp" -B "$CPP_BUILD_DIR" -DCMAKE_BUILD_TYPE=Release

echo "[2/4] Building native parity binary"
"$CMAKE_BIN" --build "$CPP_BUILD_DIR" --target web-ifc-parity -j 8

echo "[3/4] Generating parity JSON"
"$CPP_BUILD_DIR/web-ifc-parity" "$IFC_PATH" > "$CPP_JSON"
"$DOTNET_CMD" run --no-launch-profile --project "$ROOT_DIR/csharp/BazanCDE.Parsing/BazanCDE.Parsing.csproj" -- --parity-json "$IFC_PATH" > "$CS_JSON"

echo "[4/4] Comparing parity results"
python3 - "$CPP_JSON" "$CS_JSON" <<'PY'
import json
import sys

cpp_path, csharp_path = sys.argv[1:3]

def load_first_json_object(path: str):
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end < start:
        raise ValueError(f"Could not find JSON object in {path}")

    return json.loads(text[start : end + 1])

cpp = load_first_json_object(cpp_path)
csharp = load_first_json_object(csharp_path)

cpp_map = {int(m["expressId"]): int(m["triangleCount"]) for m in cpp.get("meshes", [])}
csharp_map = {int(m["expressId"]): int(m["triangleCount"]) for m in csharp.get("meshes", [])}

all_ids = sorted(set(cpp_map) | set(csharp_map))
mismatches = []
for express_id in all_ids:
    cpp_tri = cpp_map.get(express_id, 0)
    csharp_tri = csharp_map.get(express_id, 0)
    if cpp_tri != csharp_tri:
        mismatches.append((express_id, cpp_tri, csharp_tri))

cpp_total = int(cpp.get("triangleCount", 0))
csharp_total = int(csharp.get("triangleCount", 0))

print(f"C++    meshCount={len(cpp_map)} triangleCount={cpp_total}")
print(f"C#     meshCount={len(csharp_map)} triangleCount={csharp_total}")

if not mismatches:
    print("PARITY OK: no triangle-count mismatches by expressId.")
    sys.exit(0)

print(f"PARITY MISMATCHES: {len(mismatches)}")
for express_id, cpp_tri, csharp_tri in mismatches[:200]:
    print(f"  #{express_id}: cpp={cpp_tri}, csharp={csharp_tri}")

if len(mismatches) > 200:
    print(f"  ... and {len(mismatches) - 200} more")

sys.exit(1)
PY
