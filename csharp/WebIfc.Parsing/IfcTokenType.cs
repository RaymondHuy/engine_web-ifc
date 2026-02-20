namespace WebIfc.Parsing;

public enum IfcTokenType : byte
{
    Unknown = 0,
    String = 1,
    Label = 2,
    Enum = 3,
    Real = 4,
    Ref = 5,
    Empty = 6,
    SetBegin = 7,
    SetEnd = 8,
    LineEnd = 9,
    Integer = 10
}
