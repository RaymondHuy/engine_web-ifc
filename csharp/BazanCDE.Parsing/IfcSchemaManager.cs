namespace BazanCDE.Parsing;

public sealed class IfcSchemaManager : IIfcSchemaManager
{
    private readonly DefaultIfcSchemaManager _inner = new();

    public IReadOnlyList<IfcSchema> GetAvailableSchemas() => _inner.GetAvailableSchemas();

    public string GetSchemaName(IfcSchema schema) => _inner.GetSchemaName(schema);

    public uint IfcTypeToTypeCode(string name) => _inner.IfcTypeToTypeCode(name);

    public string IfcTypeCodeToType(uint typeCode) => _inner.IfcTypeCodeToType(typeCode);

    public bool IsIfcElement(uint typeCode) => _inner.IsIfcElement(typeCode);

    public IReadOnlySet<uint> GetIfcElementList() => _inner.GetIfcElementList();
}
