#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
SRC_TS="$ROOT_DIR/src/ts/ifc-schema.ts"
OUT_CS="$ROOT_DIR/csharp/BazanCDE.Parsing/IfcTypeNameMap.cs"
TMP_ENTRIES="$(mktemp)"

awk '/^export const [A-Z0-9_]+ = [0-9]+;/ {
  name=$3; val=$5; gsub(";", "", val);
  printf("            [%su] = \"%s\",\n", val, name)
}' "$SRC_TS" > "$TMP_ENTRIES"

{
  echo "namespace BazanCDE.Parsing;"
  echo
  echo "internal static class IfcTypeNameMap"
  echo "{"
  echo "    public static readonly IReadOnlyDictionary<uint, string> CodeToName = new Dictionary<uint, string>"
  echo "    {"
  cat "$TMP_ENTRIES"
  echo "    };"
  echo "}"
} > "$OUT_CS"

rm -f "$TMP_ENTRIES"
echo "Generated: $OUT_CS"
