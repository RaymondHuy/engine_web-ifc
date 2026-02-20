using System.Globalization;
using System.Linq;
using System.Text;

namespace WebIfc.Parsing;

public static class IfcStringParsing
{
    private static readonly Encoding Latin1 = Encoding.Latin1;
    private static readonly Encoding[] Iso8859Encodings;
    private static readonly Encoding? MacRoman;

    static IfcStringParsing()
    {
        try
        {
            Encoding.RegisterProvider(CodePagesEncodingProvider.Instance);
        }
        catch
        {
            // Ignore: provider may already be registered.
        }

        Iso8859Encodings = Enumerable.Range(1, 9)
            .Select(page =>
            {
                try
                {
                    return Encoding.GetEncoding($"iso-8859-{page}");
                }
                catch
                {
                    return Latin1;
                }
            })
            .ToArray();

        try
        {
            MacRoman = Encoding.GetEncoding(10000);
        }
        catch
        {
            MacRoman = null;
        }
    }

    public static string P21Encode(string input)
    {
        ArgumentNullException.ThrowIfNull(input);

        var output = new StringBuilder(input.Length + 16);
        var tmp = new StringBuilder();
        var inEncode = false;

        foreach (var c in input)
        {
            if (c > 126 || c < 32)
            {
                if (!inEncode)
                {
                    inEncode = true;
                    tmp.Clear();
                    tmp.Append(c);
                    continue;
                }

                tmp.Append(c);
                continue;
            }

            if (inEncode)
            {
                EncodeCharacters(output, tmp.ToString());
                inEncode = false;
                tmp.Clear();
            }
            else if (c == '\'')
            {
                output.Append("''");
                continue;
            }

            output.Append(c);
        }

        if (inEncode)
        {
            EncodeCharacters(output, tmp.ToString());
        }

        return output.ToString();
    }

    public static string P21Decode(string input)
    {
        ArgumentNullException.ThrowIfNull(input);
        return P21Decode(input.AsSpan());
    }

    public static string P21Decode(ReadOnlySpan<char> input)
    {
        var decoder = new P21Decoder(input.ToString());
        return decoder.Error ? string.Empty : decoder.GetResult();
    }

    private static void EncodeCharacters(StringBuilder output, string data)
    {
        output.Append("\\X2\\");
        foreach (var c in data)
        {
            output.Append(((int)c).ToString("X4", CultureInfo.InvariantCulture));
        }

        output.Append("\\X0\\");
    }

    private sealed class P21Decoder
    {
        private readonly string _input;
        private readonly StringBuilder _result = new();
        private int _index;
        private int _codepage;
        private bool _foundRoman;

        public bool Error { get; private set; }

        public P21Decoder(string input)
        {
            _input = input;
            _index = 0;
            _codepage = 0;
            _foundRoman = false;

            if (_input.Length == 0)
            {
                return;
            }

            for (var c = _input[_index]; c != '\0' && !Error; c = GetNext())
            {
                switch (c)
                {
                    case '\'':
                    {
                        var next = GetNext();
                        if (next == '\0') return;
                        if (next == '\'') _result.Append(next);
                        else Error = true;
                        break;
                    }
                    case '\\':
                    {
                        var token = GetNext();
                        if (token == '\0') return;

                        switch (token)
                        {
                            case '\\':
                                _result.Append('\\');
                                break;
                            case 'X':
                                DecodeX();
                                break;
                            case 'S':
                                DecodeS();
                                break;
                            case 'P':
                                DecodeP();
                                break;
                            default:
                                Error = true;
                                break;
                        }

                        break;
                    }
                    default:
                        _result.Append(c);
                        break;
                }
            }
        }

        public string GetResult() => _result.ToString();

        private void DecodeX()
        {
            var mode = GetNext();
            if (mode == '\0') return;

            switch (mode)
            {
                case '\\':
                {
                    var d1 = GetNextHex();
                    var d2 = GetNextHex();
                    var b = (byte)((d1 << 4) | d2);

                    if (b >= 0x80 && b <= 0x9F)
                    {
                        _foundRoman = true;
                    }

                    if (_foundRoman && MacRoman != null)
                    {
                        _result.Append(MacRoman.GetString(new[] { b }));
                    }
                    else
                    {
                        _result.Append((char)b);
                    }

                    break;
                }
                case '2':
                    ParseX(2);
                    break;
                case '4':
                    ParseX(4);
                    break;
                default:
                    Error = true;
                    break;
            }
        }

        private void DecodeS()
        {
            if (GetNext() != '\\')
            {
                Error = true;
                return;
            }

            var symbol = GetNext();
            if (symbol == '\0')
            {
                return;
            }

            _result.Append(Iso8859ToUtf(symbol));
        }

        private void DecodeP()
        {
            var c1 = GetNext();
            if (c1 == '\0') return;

            if (GetNext() != '\\')
            {
                Error = true;
                return;
            }

            if (c1 < 'A' || c1 > 'I')
            {
                Error = true;
                return;
            }

            _codepage = c1 - 'A';
        }

        private void ParseX(int bytesPerCodepoint)
        {
            if (GetNext() != '\\')
            {
                Error = true;
                return;
            }

            var bytes = new List<byte>();
            while (true)
            {
                var c = GetNext();
                if (c == '\0')
                {
                    Error = true;
                    return;
                }

                if (c == '\\')
                {
                    if (GetNext() == 'X' && GetNext() == '0' && GetNext() == '\\')
                    {
                        break;
                    }

                    Error = true;
                    return;
                }

                _index--;
                var d1 = GetNextHex();
                var d2 = GetNextHex();
                bytes.Add((byte)((d1 << 4) | d2));
            }

            if (bytesPerCodepoint == 2)
            {
                if (bytes.Count % 2 != 0)
                {
                    Error = true;
                    return;
                }

                for (var i = 0; i < bytes.Count; i += 2)
                {
                    var codeUnit = (ushort)((bytes[i] << 8) | bytes[i + 1]);
                    _result.Append((char)codeUnit);
                }

                return;
            }

            if (bytes.Count % 4 != 0)
            {
                Error = true;
                return;
            }

            for (var i = 0; i < bytes.Count; i += 4)
            {
                var codepoint =
                    (bytes[i] << 24) |
                    (bytes[i + 1] << 16) |
                    (bytes[i + 2] << 8) |
                    bytes[i + 3];

                if (codepoint < 0 || codepoint > 0x10FFFF)
                {
                    Error = true;
                    return;
                }

                _result.Append(char.ConvertFromUtf32(codepoint));
            }
        }

        private string Iso8859ToUtf(char symbol)
        {
            var b = (byte)symbol;
            var high = unchecked((byte)(b + 0x80));

            var page = _codepage;
            if (page < 0 || page >= Iso8859Encodings.Length)
            {
                page = 0;
            }

            return Iso8859Encodings[page].GetString(new[] { high });
        }

        private int GetNextHex()
        {
            var value = GetNext();
            if (value >= '0' && value <= '9') return value - '0';
            if (value >= 'A' && value <= 'F') return value - 'A' + 10;
            return 0;
        }

        private char GetNext()
        {
            _index++;
            if (_index < _input.Length)
            {
                return _input[_index];
            }

            return '\0';
        }
    }
}
