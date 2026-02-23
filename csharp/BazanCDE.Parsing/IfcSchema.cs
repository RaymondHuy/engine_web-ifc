namespace BazanCDE.Parsing;

public enum IfcSchema
{
    IFC2X3 = 0,
    IFC4 = 1,
    IFC4X3 = 2
}

public interface IIfcSchemaManager
{
    IReadOnlyList<IfcSchema> GetAvailableSchemas();
    string GetSchemaName(IfcSchema schema);
    uint IfcTypeToTypeCode(string name);
    string IfcTypeCodeToType(uint typeCode);
    bool IsIfcElement(uint typeCode);
    IReadOnlySet<uint> GetIfcElementList();
}
