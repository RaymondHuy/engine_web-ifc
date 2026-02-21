using System.Globalization;
using System.IO.MemoryMappedFiles;
using System.Linq;
using System.Text;

namespace BazanCDE.Parsing;

public sealed class IfcLoader : IDisposable
{
    private sealed class IfcLine
    {
        public uint IfcType;
        public uint TapeOffset;
    }

    private uint _maxExpressId;
    private readonly uint _lineWriterBuffer;
    private readonly IIfcSchemaManager _schemaManager;
    private readonly IfcTokenStream _tokenStream;

    private readonly Dictionary<uint, IfcLine> _lines = new();
    private readonly List<IfcLine> _headerLines = new();
    private readonly Dictionary<uint, List<uint>> _ifcTypeToExpressId = new();

    private readonly uint _fileDescriptionType;
    private readonly uint _fileNameType;
    private readonly uint _fileSchemaType;

    // Owned memory-map resources used by the LoadFile(string) overload.
    private MemoryMappedFile? _mappedFile;
    private MemoryMappedViewAccessor? _mappedViewAccessor;

    public IfcLoader(uint tapeSize, ulong memoryLimit, uint lineWriterBuffer, IIfcSchemaManager schemaManager)
    {
        ArgumentNullException.ThrowIfNull(schemaManager);

        _lineWriterBuffer = lineWriterBuffer;
        _schemaManager = schemaManager;

        var maxChunks = memoryLimit > 0 ? memoryLimit / tapeSize : 0;
        _tokenStream = new IfcTokenStream((int)tapeSize, maxChunks);
        _maxExpressId = 0;

        _fileDescriptionType = _schemaManager.IfcTypeToTypeCode("FILE_DESCRIPTION");
        _fileNameType = _schemaManager.IfcTypeToTypeCode("FILE_NAME");
        _fileSchemaType = _schemaManager.IfcTypeToTypeCode("FILE_SCHEMA");
    }

    private IfcLoader(
        uint maxExpressId,
        uint lineWriterBuffer,
        IIfcSchemaManager schemaManager,
        IfcTokenStream tokenStream,
        Dictionary<uint, IfcLine> lines,
        List<IfcLine> headerLines,
        Dictionary<uint, List<uint>> ifcTypeToExpressId)
    {
        _maxExpressId = maxExpressId;
        _lineWriterBuffer = lineWriterBuffer;
        _schemaManager = schemaManager;
        _tokenStream = tokenStream;

        foreach (var kv in lines)
        {
            _lines[kv.Key] = kv.Value;
        }

        _headerLines.AddRange(headerLines);

        foreach (var kv in ifcTypeToExpressId)
        {
            _ifcTypeToExpressId[kv.Key] = new List<uint>(kv.Value);
        }

        _fileDescriptionType = _schemaManager.IfcTypeToTypeCode("FILE_DESCRIPTION");
        _fileNameType = _schemaManager.IfcTypeToTypeCode("FILE_NAME");
        _fileSchemaType = _schemaManager.IfcTypeToTypeCode("FILE_SCHEMA");
    }

    public IReadOnlyList<uint> GetHeaderLinesWithType(uint type)
    {
        var ret = new List<uint>();
        for (var i = 0; i < _headerLines.Count; i++)
        {
            if (_headerLines[i].IfcType == type)
            {
                ret.Add((uint)i);
            }
        }

        return ret;
    }

    public void LoadFile(TokenDataSource requestData)
    {
        ClearMappedSource();
        _tokenStream.SetTokenSource(requestData);
        ParseLines();
    }

    public void LoadFile(Stream requestData)
    {
        ClearMappedSource();
        _tokenStream.SetTokenSource(requestData);
        ParseLines();
    }

    public void LoadFile(string filePath)
    {
        if (string.IsNullOrWhiteSpace(filePath))
        {
            throw new ArgumentException("File path must not be empty.", nameof(filePath));
        }

        var fullPath = Path.GetFullPath(filePath);
        if (!File.Exists(fullPath))
        {
            throw new FileNotFoundException($"IFC file not found: {fullPath}", fullPath);
        }

        ClearMappedSource();

        var fileLength = new FileInfo(fullPath).Length;
        if (fileLength == 0)
        {
            _tokenStream.SetTokenSource((_, _, _) => 0);
            ParseLines();
            return;
        }

        _mappedFile = MemoryMappedFile.CreateFromFile(
            fullPath,
            FileMode.Open,
            mapName: null,
            capacity: 0,
            access: MemoryMappedFileAccess.Read);
        _mappedViewAccessor = _mappedFile.CreateViewAccessor(0, 0, MemoryMappedFileAccess.Read);

        var viewAccessor = _mappedViewAccessor;
        var sourceLength = fileLength;

        _tokenStream.SetTokenSource((dest, sourceOffset, destSize) =>
        {
            if (sourceOffset >= sourceLength)
            {
                return 0;
            }

            var bytesToRead = (int)Math.Min(destSize, sourceLength - sourceOffset);
            if (bytesToRead <= 0)
            {
                return 0;
            }

            return viewAccessor.ReadArray(sourceOffset, dest, 0, bytesToRead);
        });

        ParseLines();
    }

    public void SaveFile(Action<byte[], int> outputData, bool orderLinesByExpressId)
    {
        ArgumentNullException.ThrowIfNull(outputData);

        var output = new StringBuilder();
        output.Append("ISO-10303-21;\nHEADER;\n");
        output.Append("/******************************************************\n");
        output.Append("* STEP Physical File produced by: That Open Engine BazanCDE\n");
        output.Append("* Module: web-ifc/IfcLoader\n");
        output.Append("* Source: https://github.com/ThatOpen/engine_web-ifc\n");
        output.Append("* Issues: https://github.com/ThatOpen/engine_web-ifc/issues\n");
        output.Append("******************************************************/\n");

        uint linesWritten = 0;

        for (var z = 0; z < 2; z++)
        {
            List<IfcLine> currentLines;
            if (z == 0)
            {
                currentLines = _headerLines;
            }
            else
            {
                currentLines = _lines.Values.ToList();
            }

            if (orderLinesByExpressId)
            {
                currentLines.Sort((a, b) => a.TapeOffset.CompareTo(b.TapeOffset));
            }

            for (var i = 0; i < currentLines.Count; i++)
            {
                var line = currentLines[i];
                if (line.IfcType == 0)
                {
                    continue;
                }

                _tokenStream.MoveTo(line.TapeOffset);
                var newLine = true;
                var insideSet = false;
                var prev = IfcTokenType.Empty;

                while (!_tokenStream.IsAtEnd())
                {
                    var t = (IfcTokenType)_tokenStream.ReadByte();

                    if (t != IfcTokenType.SetEnd && t != IfcTokenType.LineEnd)
                    {
                        if (insideSet && prev != IfcTokenType.SetBegin && prev != IfcTokenType.Label && prev != IfcTokenType.LineEnd)
                        {
                            output.Append(',');
                        }
                    }

                    if (t == IfcTokenType.LineEnd)
                    {
                        output.Append(";\n");
                        break;
                    }

                    switch (t)
                    {
                        case IfcTokenType.Unknown:
                            output.Append('*');
                            break;
                        case IfcTokenType.Empty:
                            output.Append('$');
                            break;
                        case IfcTokenType.SetBegin:
                            output.Append('(');
                            insideSet = true;
                            break;
                        case IfcTokenType.SetEnd:
                            output.Append(')');
                            break;
                        case IfcTokenType.String:
                            output.Append('\'');
                            output.Append(IfcStringParsing.P21Encode(_tokenStream.ReadString()));
                            output.Append('\'');
                            break;
                        case IfcTokenType.Enum:
                            output.Append('.');
                            output.Append(_tokenStream.ReadString());
                            output.Append('.');
                            break;
                        case IfcTokenType.Ref:
                            output.Append('#');
                            output.Append(_tokenStream.ReadUInt32().ToString(CultureInfo.InvariantCulture));
                            if (newLine)
                            {
                                output.Append('=');
                            }

                            break;
                        case IfcTokenType.Label:
                        case IfcTokenType.Real:
                        case IfcTokenType.Integer:
                            output.Append(_tokenStream.ReadString());
                            break;
                    }

                    newLine = false;
                    prev = t;
                }

                linesWritten++;
                if (linesWritten > _lineWriterBuffer)
                {
                    WriteChunk(outputData, output);
                    linesWritten = 0;
                }
            }

            if (z == 0)
            {
                output.Append("ENDSEC;\nDATA;\n");
            }
        }

        output.Append("ENDSEC;\nEND-ISO-10303-21;");
        WriteChunk(outputData, output);
    }

    public void SaveFile(Stream outputData, bool orderLinesByExpressId)
    {
        ArgumentNullException.ThrowIfNull(outputData);
        SaveFile((src, srcSize) => outputData.Write(src, 0, srcSize), orderLinesByExpressId);
    }

    public IReadOnlyList<uint> GetExpressIDsWithType(uint type)
    {
        if (!_ifcTypeToExpressId.TryGetValue(type, out var ids))
        {
            return Array.Empty<uint>();
        }

        return ids;
    }

    public IfcSchema GetSchema()
    {
        var lines = GetHeaderLinesWithType(_fileSchemaType);
        if (lines.Count == 0)
        {
            return IfcSchema.IFC2X3;
        }

        MoveToHeaderLineArgument(lines[0], 0);
        var schemas = _schemaManager.GetAvailableSchemas();

        while (!_tokenStream.IsAtEnd())
        {
            var t = (IfcTokenType)_tokenStream.ReadByte();
            if (t == IfcTokenType.LineEnd)
            {
                break;
            }

            if (t == IfcTokenType.Label)
            {
                var schemaName = _tokenStream.ReadString();
                foreach (var schema in schemas)
                {
                    if (_schemaManager.GetSchemaName(schema) == schemaName)
                    {
                        return schema;
                    }
                }
            }
        }

        return IfcSchema.IFC2X3;
    }

    public uint GetMaxExpressId() => _maxExpressId;

    public bool IsValidExpressID(uint expressID)
    {
        if (expressID == 0 || expressID > _maxExpressId || !_lines.ContainsKey(expressID))
        {
            return false;
        }

        return true;
    }

    public uint GetLineType(uint expressID)
    {
        if (expressID == 0 || expressID > _maxExpressId)
        {
            return 0;
        }

        if (!_lines.TryGetValue(expressID, out var line))
        {
            return 0;
        }

        return line.IfcType;
    }

    public bool IsAtEnd() => _tokenStream.IsAtEnd();

    public void MoveToLineArgument(uint expressID, uint argumentIndex)
    {
        if (!_lines.TryGetValue(expressID, out var line))
        {
            return;
        }

        _tokenStream.MoveTo(line.TapeOffset);
        ArgumentOffset(argumentIndex);
    }

    public void MoveToHeaderLineArgument(uint lineID, uint argumentIndex)
    {
        if (lineID >= _headerLines.Count)
        {
            return;
        }

        _tokenStream.MoveTo(_headerLines[(int)lineID].TapeOffset);
        ArgumentOffset(argumentIndex);
    }

    public string GetStringArgument()
    {
        _tokenStream.ReadByte();
        return _tokenStream.ReadString();
    }

    public string GetDecodedStringArgument()
    {
        var str = GetStringArgument();
        return IfcStringParsing.P21Decode(str);
    }

    public string GetExpandedUUIDArgument()
    {
        return IfcGuidUtils.ExpandIfcGuid(GetStringArgument());
    }

    public double GetDoubleArgument()
    {
        var str = GetStringArgument();
        return double.Parse(str, NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    public long GetIntArgument()
    {
        var str = GetStringArgument();
        return long.Parse(str, NumberStyles.Integer, CultureInfo.InvariantCulture);
    }

    public long GetIntArgument(uint tapeOffset)
    {
        _tokenStream.MoveTo(tapeOffset);
        return GetIntArgument();
    }

    public double GetDoubleArgument(uint tapeOffset)
    {
        _tokenStream.MoveTo(tapeOffset);
        return GetDoubleArgument();
    }

    public string GetDoubleArgumentAsString() => GetStringArgument();

    public uint GetRefArgument()
    {
        if ((IfcTokenType)_tokenStream.ReadByte() != IfcTokenType.Ref)
        {
            return 0;
        }

        return _tokenStream.ReadUInt32();
    }

    public uint GetRefArgument(uint tapeOffset)
    {
        _tokenStream.MoveTo(tapeOffset);
        return GetRefArgument();
    }

    public uint GetOptionalRefArgument()
    {
        var t = GetTokenType();
        if (t == IfcTokenType.Empty)
        {
            return 0;
        }

        if (t == IfcTokenType.Ref)
        {
            return _tokenStream.ReadUInt32();
        }

        return 0;
    }

    public IfcTokenType GetTokenType()
    {
        return (IfcTokenType)_tokenStream.ReadByte();
    }

    public IfcTokenType GetTokenType(uint tapeOffset)
    {
        _tokenStream.MoveTo(tapeOffset);
        return GetTokenType();
    }

    public IReadOnlyList<uint> GetSetArgument()
    {
        var tapeOffsets = new List<uint>(4);

        _tokenStream.ReadByte();
        var depth = 1;

        while (depth > 0)
        {
            var offset = (uint)_tokenStream.GetReadOffset();
            var t = (IfcTokenType)_tokenStream.ReadByte();

            switch (t)
            {
                case IfcTokenType.SetBegin:
                    depth++;
                    break;
                case IfcTokenType.SetEnd:
                    depth--;
                    break;
                case IfcTokenType.Ref:
                    tapeOffsets.Add(offset);
                    _tokenStream.ReadUInt32();
                    break;
                case IfcTokenType.String:
                case IfcTokenType.Integer:
                case IfcTokenType.Real:
                case IfcTokenType.Label:
                case IfcTokenType.Enum:
                {
                    tapeOffsets.Add(offset);
                    var length = _tokenStream.ReadUInt16();
                    _tokenStream.Forward(length);
                    break;
                }
            }
        }

        return tapeOffsets;
    }

    public IReadOnlyList<IReadOnlyList<uint>> GetSetListArgument()
    {
        var tapeOffsets = new List<IReadOnlyList<uint>>();
        _tokenStream.ReadByte();
        var depth = 1;
        var tempSet = new List<uint>();

        while (true)
        {
            var offset = (uint)_tokenStream.GetReadOffset();
            var t = (IfcTokenType)_tokenStream.ReadByte();

            if (t == IfcTokenType.SetBegin)
            {
                tempSet = new List<uint>();
                depth++;
            }
            else if (t == IfcTokenType.SetEnd)
            {
                if (tempSet.Count > 0)
                {
                    tapeOffsets.Add(tempSet);
                    tempSet = new List<uint>();
                }

                depth--;
            }
            else
            {
                tempSet.Add(offset);

                if (t == IfcTokenType.Ref)
                {
                    _tokenStream.ReadUInt32();
                }
                else if (t is IfcTokenType.String or IfcTokenType.Integer or IfcTokenType.Real or IfcTokenType.Label or IfcTokenType.Enum)
                {
                    var length = _tokenStream.ReadUInt16();
                    _tokenStream.Forward(length);
                }
            }

            if (depth == 0)
            {
                break;
            }
        }

        return tapeOffsets;
    }

    public void MoveToArgumentOffset(uint expressID, uint argumentIndex)
    {
        if (!_lines.TryGetValue(expressID, out var line))
        {
            return;
        }

        _tokenStream.MoveTo(line.TapeOffset);
        ArgumentOffset(argumentIndex);
    }

    public uint GetNoLineArguments(uint expressID)
    {
        if (!_lines.TryGetValue(expressID, out var line))
        {
            return 0;
        }

        _tokenStream.MoveTo(line.TapeOffset);
        _tokenStream.ReadByte();
        _tokenStream.ReadUInt32();
        _tokenStream.ReadByte();
        var length = _tokenStream.ReadUInt16();
        _tokenStream.Forward(length);
        _tokenStream.ReadByte();

        uint noArguments = 0;

        while (true)
        {
            var t = (IfcTokenType)_tokenStream.ReadByte();
            if (t is IfcTokenType.SetEnd or IfcTokenType.LineEnd)
            {
                return noArguments;
            }

            if (t is IfcTokenType.Unknown or IfcTokenType.Empty)
            {
                noArguments++;
                continue;
            }

            if (t == IfcTokenType.SetBegin)
            {
                StepBack();
                GetSetArgument();
                noArguments++;
                continue;
            }

            if (t is IfcTokenType.String or IfcTokenType.Integer or IfcTokenType.Real or IfcTokenType.Label or IfcTokenType.Enum)
            {
                length = _tokenStream.ReadUInt16();
                _tokenStream.Forward(length);
                noArguments++;
                if (t == IfcTokenType.Label)
                {
                    GetSetArgument();
                }

                continue;
            }

            if (t == IfcTokenType.Ref)
            {
                _tokenStream.ReadUInt32();
                noArguments++;
            }
        }
    }

    public void StepBack()
    {
        _tokenStream.Back();
    }

    public double GetOptionalDoubleParam(double defaultValue = 0)
    {
        var tk = GetTokenType();
        if (tk == IfcTokenType.Real)
        {
            StepBack();
            return GetDoubleArgument();
        }

        if (tk == IfcTokenType.Integer)
        {
            StepBack();
            return GetIntArgument();
        }

        return defaultValue;
    }

    public IReadOnlyList<uint> GetAllLines()
    {
        return _lines.Keys.ToList();
    }

    public uint GetNextExpressID(uint expressId)
    {
        var currentId = expressId + 1;
        while (!_lines.ContainsKey(currentId))
        {
            if (currentId == uint.MaxValue)
            {
                return 0;
            }

            currentId++;
        }

        return currentId;
    }

    public uint GetCurrentLineExpressID()
    {
        if (_lines.Count == 0)
        {
            return 0;
        }

        var pos = (uint)_tokenStream.GetReadOffset();
        uint bestExpressId = 0;
        uint bestOffset = 0;

        foreach (var kv in _lines)
        {
            var offset = kv.Value.TapeOffset;
            if (offset <= pos && offset >= bestOffset)
            {
                bestOffset = offset;
                bestExpressId = kv.Key;
            }
        }

        return bestExpressId;
    }

    public void Push(ReadOnlySpan<byte> value)
    {
        _tokenStream.Push(value);
    }

    public void PushDouble(double input)
    {
        var numberString = input.ToString("G17", CultureInfo.InvariantCulture);
        var eLoc = numberString.IndexOf('e');
        if (eLoc != -1)
        {
            numberString = numberString[..eLoc] + 'E' + numberString[(eLoc + 1)..];
        }
        else if (Math.Floor(input) == input)
        {
            numberString += ".";
        }

        var bytes = Encoding.UTF8.GetBytes(numberString);
        _tokenStream.PushUInt16((ushort)bytes.Length);
        _tokenStream.Push(bytes);
    }

    public void PushInt(int input)
    {
        var numberString = input.ToString(CultureInfo.InvariantCulture);
        var bytes = Encoding.UTF8.GetBytes(numberString);
        _tokenStream.PushUInt16((ushort)bytes.Length);
        _tokenStream.Push(bytes);
    }

    public long GetTotalSize()
    {
        return _tokenStream.GetTotalSize();
    }

    public void UpdateLineTape(uint expressID, uint type, uint start)
    {
        if (!_lines.TryGetValue(expressID, out var line))
        {
            line = new IfcLine
            {
                IfcType = type,
                TapeOffset = start
            };
            _lines[expressID] = line;

            if (!_ifcTypeToExpressId.TryGetValue(type, out var ids))
            {
                ids = new List<uint>();
                _ifcTypeToExpressId[type] = ids;
            }

            ids.Add(expressID);
            _maxExpressId = Math.Max(expressID, _maxExpressId);
            return;
        }

        line.TapeOffset = start;
    }

    public void AddHeaderLineTape(uint type, uint start)
    {
        _headerLines.Add(new IfcLine
        {
            IfcType = type,
            TapeOffset = start
        });
    }

    public void RemoveLine(uint expressID)
    {
        _lines.Remove(expressID);
    }

    public string GenerateUUID()
    {
        return IfcGuidUtils.CompressIfcGuid(IfcGuidUtils.GenerateStringUuid());
    }

    public IfcLoader Clone()
    {
        return new IfcLoader(_maxExpressId, _lineWriterBuffer, _schemaManager, _tokenStream.Clone(), _lines, _headerLines, _ifcTypeToExpressId);
    }

    public void Dispose()
    {
        ClearMappedSource();
        _tokenStream.Dispose();
        _lines.Clear();
        _headerLines.Clear();
        _ifcTypeToExpressId.Clear();
    }

    private void ClearMappedSource()
    {
        _mappedViewAccessor?.Dispose();
        _mappedViewAccessor = null;

        _mappedFile?.Dispose();
        _mappedFile = null;
    }

    private void ParseLines()
    {
        _lines.Clear();
        _headerLines.Clear();
        _ifcTypeToExpressId.Clear();
        _maxExpressId = 0;

        uint currentIfcType = 0;
        uint currentExpressID = 0;
        uint currentTapeOffset = 0;

        while (!_tokenStream.IsAtEnd())
        {
            var t = (IfcTokenType)_tokenStream.ReadByte();
            switch (t)
            {
                case IfcTokenType.LineEnd:
                {
                    if (currentIfcType != 0)
                    {
                        var line = new IfcLine
                        {
                            IfcType = currentIfcType,
                            TapeOffset = currentTapeOffset
                        };

                        if (currentIfcType == _fileDescriptionType || currentIfcType == _fileNameType || currentIfcType == _fileSchemaType)
                        {
                            _headerLines.Add(line);
                        }
                        else if (currentExpressID != 0)
                        {
                            if (!_ifcTypeToExpressId.TryGetValue(currentIfcType, out var ids))
                            {
                                ids = new List<uint>();
                                _ifcTypeToExpressId[currentIfcType] = ids;
                            }

                            ids.Add(currentExpressID);
                            _maxExpressId = Math.Max(_maxExpressId, currentExpressID);
                            _lines[currentExpressID] = line;
                            currentExpressID = 0;
                        }

                        currentIfcType = 0;
                    }

                    currentTapeOffset = (uint)_tokenStream.GetReadOffset();
                    break;
                }
                case IfcTokenType.Unknown:
                case IfcTokenType.Empty:
                case IfcTokenType.SetBegin:
                case IfcTokenType.SetEnd:
                    break;
                case IfcTokenType.String:
                case IfcTokenType.Real:
                case IfcTokenType.Integer:
                case IfcTokenType.Enum:
                {
                    var size = _tokenStream.ReadUInt16();
                    _tokenStream.Forward(size);
                    break;
                }
                case IfcTokenType.Label:
                {
                    var s = _tokenStream.ReadString();
                    if (currentIfcType == 0)
                    {
                        currentIfcType = _schemaManager.IfcTypeToTypeCode(s);
                    }

                    break;
                }
                case IfcTokenType.Ref:
                {
                    var reference = _tokenStream.ReadUInt32();
                    if (currentExpressID == 0)
                    {
                        currentExpressID = reference;
                    }

                    break;
                }
            }
        }
    }

    private void ArgumentOffset(uint argumentIndex)
    {
        uint movedOver = 0;
        uint setDepth = 0;

        while (true)
        {
            if (setDepth == 1)
            {
                movedOver++;
                if (movedOver - 1 == argumentIndex)
                {
                    return;
                }
            }

            var t = (IfcTokenType)_tokenStream.ReadByte();
            switch (t)
            {
                case IfcTokenType.LineEnd:
                    return;
                case IfcTokenType.Unknown:
                case IfcTokenType.Empty:
                    break;
                case IfcTokenType.SetBegin:
                    setDepth++;
                    break;
                case IfcTokenType.SetEnd:
                    setDepth--;
                    if (setDepth == 0)
                    {
                        return;
                    }

                    break;
                case IfcTokenType.String:
                case IfcTokenType.Enum:
                case IfcTokenType.Label:
                case IfcTokenType.Integer:
                case IfcTokenType.Real:
                {
                    var length = _tokenStream.ReadUInt16();
                    _tokenStream.Forward(length);
                    break;
                }
                case IfcTokenType.Ref:
                    _tokenStream.ReadUInt32();
                    break;
            }
        }
    }

    private static void WriteChunk(Action<byte[], int> outputData, StringBuilder output)
    {
        if (output.Length == 0)
        {
            return;
        }

        var bytes = Encoding.UTF8.GetBytes(output.ToString());
        outputData(bytes, bytes.Length);
        output.Clear();
    }
}
