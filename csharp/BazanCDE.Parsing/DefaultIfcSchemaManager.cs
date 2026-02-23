using System.Text;

namespace BazanCDE.Parsing;

public sealed class DefaultIfcSchemaManager : IIfcSchemaManager
{
    private readonly uint[] _crcTable = new uint[256];
    private readonly HashSet<uint> _ifcElements = new();
    private static readonly IReadOnlyList<IfcSchema> Schemas = new[]
    {
        IfcSchema.IFC2X3,
        IfcSchema.IFC4,
        IfcSchema.IFC4X3
    };

    private static readonly IReadOnlyDictionary<IfcSchema, string> SchemaNames = new Dictionary<IfcSchema, string>
    {
        [IfcSchema.IFC2X3] = "IFC2X3",
        [IfcSchema.IFC4] = "IFC4",
        [IfcSchema.IFC4X3] = "IFC4X3"
    };

    public DefaultIfcSchemaManager()
    {
        for (uint n = 0; n < 256; n++)
        {
            var c = n;
            for (var k = 0; k < 8; k++)
            {
                c = ((c & 1) != 0) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }

            _crcTable[n] = c;
        }

        foreach (var ifcElementName in IfcElementTypeNames.Names)
        {
            var code = IfcTypeToTypeCode(ifcElementName);
            if (code != 0)
            {
                _ifcElements.Add(code);
            }
        }
    }

    public IReadOnlyList<IfcSchema> GetAvailableSchemas() => Schemas;

    public string GetSchemaName(IfcSchema schema)
    {
        return SchemaNames.TryGetValue(schema, out var name) ? name : string.Empty;
    }

    public uint IfcTypeToTypeCode(string name)
    {
        if (string.IsNullOrEmpty(name))
        {
            return 0;
        }

        var hash = ComputeCrc32(name);
        if (IfcTypeNameMap.CodeToName.ContainsKey(hash))
        {
            return hash;
        }

        // Fallback for mixed/camel case input (e.g. IfcWall)
        var upperHash = ComputeCrc32(name.ToUpperInvariant());
        return upperHash;
    }

    public string IfcTypeCodeToType(uint typeCode)
    {
        return IfcTypeNameMap.CodeToName.TryGetValue(typeCode, out var name)
            ? name
            : string.Empty;
    }

    public bool IsIfcElement(uint typeCode)
    {
        return _ifcElements.Contains(typeCode);
    }

    public IReadOnlySet<uint> GetIfcElementList()
    {
        return _ifcElements;
    }

    private uint ComputeCrc32(string value)
    {
        var bytes = Encoding.ASCII.GetBytes(value);
        uint c = 0xFFFFFFFFu;
        for (var i = 0; i < bytes.Length; i++)
        {
            c = _crcTable[(c ^ bytes[i]) & 0xFF] ^ (c >> 8);
        }

        return c ^ 0xFFFFFFFFu;
    }
}
