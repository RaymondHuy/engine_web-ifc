using System.Buffers.Binary;
using System.Text;

namespace BazanCDE.Parsing;

public delegate int TokenDataSource(byte[] dest, long sourceOffset, int destSize);

public sealed class IfcTokenStream : IDisposable
{
    private long _readPtr;
    private int _currentChunk;
    private ulong _activeChunks;
    private int _chunkSize;
    private readonly ulong _maxChunks;
    private readonly List<IfcTokenChunk> _chunks = new();

    private IfcTokenChunk? _tokenChunk;
    private IfcFileStream? _fileStream;

    public IfcTokenStream(int chunkSize, ulong maxChunks)
    {
        _chunkSize = chunkSize;
        _maxChunks = maxChunks;
    }

    public long GetNoLines() => _fileStream?.GetNoLines() ?? 0;

    public void SetTokenSource(TokenDataSource requestData)
    {
        ArgumentNullException.ThrowIfNull(requestData);

        ResetForNewSource();
        _fileStream = new IfcFileStream(requestData, _chunkSize);

        long tokenOffset = 0;
        while (!_fileStream.IsAtEnd())
        {
            CheckMemory();
            var chunk = new IfcTokenChunk(_chunkSize, tokenOffset, _fileStream.GetRef(), _fileStream);
            var cSize = chunk.TokenSize;
            tokenOffset += cSize;

            if (cSize > _chunkSize)
            {
                _chunkSize = cSize;
            }

            _chunks.Add(chunk);
            _activeChunks++;
        }

        if (_chunks.Count > 0)
        {
            _tokenChunk = _chunks[0];
        }

        _fileStream.Clear();
        _readPtr = 0;
        _currentChunk = 0;
    }

    public void SetTokenSource(Stream requestData)
    {
        ArgumentNullException.ThrowIfNull(requestData);
        SetTokenSource((dest, sourceOffset, destSize) =>
        {
            requestData.Seek(sourceOffset, SeekOrigin.Begin);
            return requestData.Read(dest, 0, destSize);
        });
    }

    public byte ReadByte()
    {
        EnsureChunkLoaded();
        var value = _tokenChunk!.ReadByte((int)_readPtr);
        Forward(1);
        return value;
    }

    public ushort ReadUInt16()
    {
        EnsureChunkLoaded();
        var value = _tokenChunk!.ReadUInt16((int)_readPtr);
        Forward(2);
        return value;
    }

    public uint ReadUInt32()
    {
        EnsureChunkLoaded();
        var value = _tokenChunk!.ReadUInt32((int)_readPtr);
        Forward(4);
        return value;
    }

    public string ReadString()
    {
        EnsureChunkLoaded();

        var length = _tokenChunk!.ReadUInt16((int)_readPtr);
        Forward(2);
        if (length <= 0)
        {
            return string.Empty;
        }

        var value = _tokenChunk.ReadString((int)_readPtr, length);
        Forward(length);
        return value;
    }

    public void PushByte(byte value)
    {
        Span<byte> data = stackalloc byte[1];
        data[0] = value;
        Push(data);
    }

    public void PushUInt16(ushort value)
    {
        Span<byte> data = stackalloc byte[2];
        BinaryPrimitives.WriteUInt16LittleEndian(data, value);
        Push(data);
    }

    public void PushUInt32(uint value)
    {
        Span<byte> data = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32LittleEndian(data, value);
        Push(data);
    }

    public void Push(ReadOnlySpan<byte> value)
    {
        if (_chunks.Count == 0)
        {
            _chunks.Add(new IfcTokenChunk(_chunkSize, 0, 0, _fileStream));
            _activeChunks++;
        }

        var last = _chunks[^1];
        if (last.TokenSize + value.Length > last.MaxSize)
        {
            CheckMemory();
            var fsRef = _fileStream?.GetRef() ?? 0;
            _chunks.Add(new IfcTokenChunk(_chunkSize, last.TokenRef + last.TokenSize, fsRef, _fileStream));
            _activeChunks++;
            last = _chunks[^1];
        }

        last.Push(value);
    }

    public void Forward(int size)
    {
        if (_chunks.Count == 0 || _tokenChunk == null)
        {
            return;
        }

        _readPtr += size;
        while (_readPtr >= _tokenChunk.TokenSize)
        {
            if (_currentChunk == _chunks.Count - 1)
            {
                _readPtr = _chunks[^1].TokenSize;
                return;
            }

            _readPtr -= _tokenChunk.TokenSize;
            _currentChunk++;
            _tokenChunk = _chunks[_currentChunk];
        }
    }

    public void MoveTo(long pos)
    {
        if (_chunks.Count == 0)
        {
            _currentChunk = 0;
            _tokenChunk = null;
            _readPtr = 0;
            return;
        }

        for (var i = _chunks.Count - 1; i >= 0; i--)
        {
            if (_chunks[i].TokenRef <= pos)
            {
                _currentChunk = i;
                _tokenChunk = _chunks[_currentChunk];
                _readPtr = pos - _tokenChunk.TokenRef;
                break;
            }
        }
    }

    public void Back()
    {
        if (_chunks.Count == 0 || _tokenChunk == null)
        {
            return;
        }

        if (_readPtr == 0)
        {
            if (_currentChunk > 0)
            {
                _tokenChunk = _chunks[--_currentChunk];
                _readPtr = _tokenChunk.TokenSize - 1;
                return;
            }
        }

        _readPtr--;
    }

    public bool IsAtEnd()
    {
        if (_chunks.Count == 0)
        {
            return true;
        }

        return _currentChunk >= _chunks.Count - 1 && _readPtr >= _chunks[^1].TokenSize;
    }

    public long GetReadOffset()
    {
        if (_tokenChunk == null)
        {
            return 0;
        }

        return _tokenChunk.TokenRef + _readPtr;
    }

    public long GetTotalSize()
    {
        if (_chunks.Count == 0)
        {
            return 0;
        }

        return _chunks[^1].TokenRef + _chunks[^1].TokenSize;
    }

    public IfcTokenStream Clone()
    {
        var newStream = new IfcTokenStream(_activeChunks, _maxChunks, _chunks, _fileStream?.Clone());
        return newStream;
    }

    public void Dispose()
    {
        foreach (var chunk in _chunks)
        {
            chunk.Clear(force: true);
        }

        _chunks.Clear();
        _fileStream?.Dispose();
        _fileStream = null;
        _tokenChunk = null;
    }

    private IfcTokenStream(ulong activeChunks, ulong maxChunks, List<IfcTokenChunk> chunks, IfcFileStream? fileStream)
    {
        _activeChunks = activeChunks;
        _maxChunks = maxChunks;
        _chunks.AddRange(chunks);
        _fileStream = fileStream;
        _tokenChunk = _chunks.Count > 0 ? _chunks[0] : null;
        _currentChunk = 0;
        _readPtr = 0;
        _chunkSize = _chunks.Count > 0 ? _chunks[0].MaxSize : 0;
    }

    private void EnsureChunkLoaded()
    {
        if (_tokenChunk == null)
        {
            throw new InvalidOperationException("Token stream is not initialized.");
        }

        if (!_tokenChunk.IsLoaded)
        {
            CheckMemory();
            _activeChunks++;
            _tokenChunk.EnsureLoaded();
        }
    }

    private void CheckMemory()
    {
        if (_maxChunks == 0 || _activeChunks != _maxChunks)
        {
            return;
        }

        for (var i = 0; i < _chunks.Count; i++)
        {
            if (_chunks[i].IsLoaded && _chunks[i].Clear(force: false))
            {
                _activeChunks--;
                break;
            }
        }
    }

    private void ResetForNewSource()
    {
        foreach (var chunk in _chunks)
        {
            chunk.Clear(force: true);
        }

        _chunks.Clear();
        _tokenChunk = null;
        _currentChunk = 0;
        _readPtr = 0;
        _activeChunks = 0;
        _fileStream?.Dispose();
        _fileStream = null;
    }

    private sealed class IfcFileStream : IDisposable
    {
        private readonly TokenDataSource _dataSource;
        private long _pointer;
        private readonly int _size;
        private byte _prev;
        private long _currentSize;
        private long _startRef;
        private byte[]? _buffer;
        private long _noLines;

        public IfcFileStream(TokenDataSource requestData, int size)
        {
            _dataSource = requestData;
            _size = size;

            var countBuffer = new byte[_size];
            long countSize;
            long startCountRef = 0;
            while ((countSize = _dataSource(countBuffer, startCountRef, _size)) != 0)
            {
                for (var i = 0; i < countSize; i++)
                {
                    if (countBuffer[i] == (byte)'\n') _noLines++;
                }

                startCountRef += countSize;
            }

            Load();
        }

        public long GetNoLines() => _noLines;

        public void Go(long reference)
        {
            _startRef = reference;
            Load();
        }

        public void Forward()
        {
            _pointer++;
            if (_pointer == _currentSize && _currentSize != 0)
            {
                _startRef += _currentSize;
                Load();
            }
        }

        public void Back()
        {
            if (_pointer == 0)
            {
                if (_startRef > 0)
                {
                    _startRef--;
                    Load();
                    _pointer = 0;
                }

                return;
            }

            _pointer--;
        }

        public void Clear()
        {
            _buffer = null;
        }

        public byte Prev()
        {
            if (_pointer == 0) return _prev;
            return _buffer![_pointer - 1];
        }

        public bool IsAtEnd()
        {
            return _pointer == _currentSize && _currentSize == 0;
        }

        public long GetRef()
        {
            return _startRef + _pointer;
        }

        public byte Get()
        {
            return _buffer![_pointer];
        }

        public IfcFileStream Clone()
        {
            return new IfcFileStream(_dataSource, _size);
        }

        public void Dispose()
        {
            _buffer = null;
        }

        private void Load()
        {
            if (_buffer == null)
            {
                _buffer = new byte[_size];
            }
            else if (_currentSize > 0)
            {
                _prev = _buffer[_currentSize - 1];
            }

            _currentSize = _dataSource(_buffer, _startRef, _size);
            _pointer = 0;
        }
    }

    private sealed class IfcTokenChunk
    {
        private bool _loaded;
        private int _currentSize;
        private readonly long _startRef;
        private readonly long _fileStartRef;
        private int _chunkSize;
        private byte[]? _chunkData;
        private readonly IfcFileStream? _fileStream;

        public IfcTokenChunk(int chunkSize, long startRef, long fileStartRef, IfcFileStream? fileStream)
        {
            _startRef = startRef;
            _fileStartRef = fileStartRef;
            _chunkSize = chunkSize;
            _fileStream = fileStream;
            _chunkData = null;
            _loaded = true;
            _currentSize = 0;
            if (_fileStream != null)
            {
                Load();
            }
        }

        public bool IsLoaded => _loaded;
        public int TokenSize => _currentSize;
        public long TokenRef => _startRef;
        public int MaxSize => _chunkSize;

        public bool Clear(bool force)
        {
            if (_fileStream == null && !force)
            {
                return false;
            }

            _chunkData = null;
            _loaded = false;
            return true;
        }

        public void EnsureLoaded()
        {
            if (!_loaded)
            {
                Load();
            }
        }

        public byte ReadByte(int ptr)
        {
            EnsureLoaded();
            return _chunkData![ptr];
        }

        public ushort ReadUInt16(int ptr)
        {
            EnsureLoaded();
            return BinaryPrimitives.ReadUInt16LittleEndian(_chunkData.AsSpan(ptr, 2));
        }

        public uint ReadUInt32(int ptr)
        {
            EnsureLoaded();
            return BinaryPrimitives.ReadUInt32LittleEndian(_chunkData.AsSpan(ptr, 4));
        }

        public string ReadString(int ptr, int size)
        {
            EnsureLoaded();
            return Encoding.UTF8.GetString(_chunkData!, ptr, size);
        }

        public void Push(ReadOnlySpan<byte> value)
        {
            if (_chunkData == null)
            {
                _chunkData = new byte[_chunkSize];
            }

            _currentSize += value.Length;
            if (_currentSize > _chunkSize)
            {
                var oldData = _chunkData;
                _chunkData = new byte[_currentSize];
                oldData.AsSpan(0, _currentSize - value.Length).CopyTo(_chunkData);
                _chunkSize = _currentSize;
            }

            value.CopyTo(_chunkData.AsSpan(_currentSize - value.Length));
        }

        private void Load()
        {
            _chunkData = new byte[_chunkSize];
            _loaded = true;

            if (_fileStream!.GetRef() != _fileStartRef)
            {
                _fileStream.Go(_fileStartRef);
            }

            var temp = new List<byte>(50);
            _currentSize = 0;

            while (!_fileStream.IsAtEnd() && _currentSize < _chunkSize)
            {
                var c = (char)_fileStream.Get();

                if (c is ' ' or '\n' or '\r' or '\t')
                {
                    _fileStream.Forward();
                    continue;
                }

                if (c == '\'')
                {
                    _fileStream.Forward();
                    temp.Clear();

                    while (true)
                    {
                        temp.Add(_fileStream.Get());

                        if ((char)_fileStream.Get() == '\'')
                        {
                            _fileStream.Forward();
                            if ((char)_fileStream.Get() == '\'')
                            {
                                temp.Add(_fileStream.Get());
                            }
                            else
                            {
                                _fileStream.Back();
                                temp.RemoveAt(temp.Count - 1);
                                break;
                            }
                        }

                        _fileStream.Forward();
                    }

                    PushByte((byte)IfcTokenType.String);
                    PushUInt16((ushort)temp.Count);
                    if (temp.Count > 0)
                    {
                        Push(temp.ToArray());
                    }
                }
                else if (c == '#')
                {
                    _fileStream.Forward();
                    uint num = 0;
                    var ch = (char)_fileStream.Get();
                    while (ch is >= '0' and <= '9')
                    {
                        num = num * 10 + (uint)(ch - '0');
                        _fileStream.Forward();
                        ch = (char)_fileStream.Get();
                    }

                    PushByte((byte)IfcTokenType.Ref);
                    PushUInt32(num);
                    continue;
                }
                else if (c == '$')
                {
                    PushByte((byte)IfcTokenType.Empty);
                }
                else if (c == '*')
                {
                    if ((char)_fileStream.Prev() == '/')
                    {
                        _fileStream.Forward();
                        while (!((char)_fileStream.Prev() == '*' && (char)_fileStream.Get() == '/'))
                        {
                            _fileStream.Forward();
                        }
                    }
                    else
                    {
                        PushByte((byte)IfcTokenType.Unknown);
                    }
                }
                else if (c == '(')
                {
                    PushByte((byte)IfcTokenType.SetBegin);
                }
                else if (c is >= '0' and <= '9')
                {
                    temp.Clear();
                    if ((char)_fileStream.Prev() == '-')
                    {
                        temp.Add((byte)'-');
                    }

                    var ch = (char)_fileStream.Get();
                    var isFrac = false;
                    while ((ch is >= '0' and <= '9') || ch is '.' or 'e' or 'E' or '-' or '+')
                    {
                        temp.Add((byte)ch);
                        if (ch is '.' or 'E') isFrac = true;
                        _fileStream.Forward();
                        ch = (char)_fileStream.Get();
                    }

                    PushByte((byte)(isFrac ? IfcTokenType.Real : IfcTokenType.Integer));
                    PushUInt16((ushort)temp.Count);
                    Push(temp.ToArray());
                    continue;
                }
                else if (c == '.')
                {
                    temp.Clear();
                    _fileStream.Forward();
                    var ch = (char)_fileStream.Get();
                    while (ch != '.')
                    {
                        temp.Add((byte)ch);
                        _fileStream.Forward();
                        ch = (char)_fileStream.Get();
                    }

                    PushByte((byte)IfcTokenType.Enum);
                    PushUInt16((ushort)temp.Count);
                    Push(temp.ToArray());
                }
                else if (IsAsciiLetter(c))
                {
                    temp.Clear();
                    var ch = (char)_fileStream.Get();
                    while (IsAsciiLetter(ch) || IsAsciiDigit(ch) || ch == '_')
                    {
                        temp.Add((byte)ch);
                        _fileStream.Forward();
                        ch = (char)_fileStream.Get();
                    }

                    PushByte((byte)IfcTokenType.Label);
                    PushUInt16((ushort)temp.Count);
                    Push(temp.ToArray());
                    continue;
                }
                else if (c == ')')
                {
                    PushByte((byte)IfcTokenType.SetEnd);
                }
                else if (c == ';')
                {
                    PushByte((byte)IfcTokenType.LineEnd);
                }

                _fileStream.Forward();
            }
        }

        private void PushByte(byte value)
        {
            Span<byte> data = stackalloc byte[1];
            data[0] = value;
            Push(data);
        }

        private void PushUInt16(ushort value)
        {
            Span<byte> data = stackalloc byte[2];
            BinaryPrimitives.WriteUInt16LittleEndian(data, value);
            Push(data);
        }

        private void PushUInt32(uint value)
        {
            Span<byte> data = stackalloc byte[4];
            BinaryPrimitives.WriteUInt32LittleEndian(data, value);
            Push(data);
        }

        private static bool IsAsciiLetter(char c)
        {
            return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
        }

        private static bool IsAsciiDigit(char c)
        {
            return c >= '0' && c <= '9';
        }
    }
}
