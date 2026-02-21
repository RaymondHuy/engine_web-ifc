using System.Linq;
using System.Text;

namespace BazanCDE.Parsing;

public static class IfcGuidUtils
{
    private static readonly char[] Base16Chars = "0123456789ABCDEF".ToCharArray();
    private static readonly char[] Base64Chars =
    {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
        'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
        '_', '$'
    };

    private static readonly sbyte[] Base16Mask = BuildBase16Mask();
    private static readonly sbyte[] Base64Mask = BuildBase64Mask();

    public static string GenerateStringUuid() => Guid.NewGuid().ToString();

    public static string ExpandIfcGuid(string guid)
    {
        ArgumentNullException.ThrowIfNull(guid);
        if (guid.Length < 22)
        {
            throw new ArgumentException("IFC GUID must have at least 22 characters.", nameof(guid));
        }

        var temp = new char[33];
        var ii = 0;

        for (var i = 0; i < 22; i += 2)
        {
            var hi = Base64Mask[guid[i]];
            var lo = Base64Mask[guid[i + 1]];
            if (hi < 0 || lo < 0)
            {
                throw new FormatException("Invalid IFC GUID base64 character.");
            }

            var n = (hi << 6) + lo;
            var t = n / 16;
            temp[ii + 2] = Base16Chars[n % 16];
            temp[ii + 1] = Base16Chars[t % 16];
            temp[ii + 0] = Base16Chars[t / 16];
            ii += 3;
        }

        var hex = new string(temp, 1, 32);
        return string.Create(36, hex, static (span, h) =>
        {
            h.AsSpan(0, 8).CopyTo(span[0..8]);
            span[8] = '-';
            h.AsSpan(8, 4).CopyTo(span[9..13]);
            span[13] = '-';
            h.AsSpan(12, 4).CopyTo(span[14..18]);
            span[18] = '-';
            h.AsSpan(16, 4).CopyTo(span[19..23]);
            span[23] = '-';
            h.AsSpan(20, 12).CopyTo(span[24..36]);
        });
    }

    public static string CompressIfcGuid(string guid)
    {
        ArgumentNullException.ThrowIfNull(guid);

        var rawHex = new string(guid.Where(c => c != '-').ToArray()).ToUpperInvariant();
        if (rawHex.Length != 32)
        {
            throw new ArgumentException("GUID must contain exactly 32 hex digits (with or without dashes).", nameof(guid));
        }

        var temp = "0" + rawHex;
        var result = new char[22];

        for (int i = 0, oi = 0; i < 32; i += 3)
        {
            var a = Base16Mask[temp[i]];
            var b = Base16Mask[temp[i + 1]];
            var c = Base16Mask[temp[i + 2]];

            if (a < 0 || b < 0 || c < 0)
            {
                throw new FormatException("GUID contains non-hex characters.");
            }

            var n = (a << 8) + (b << 4) + c;
            result[oi + 1] = Base64Chars[n % 64];
            result[oi + 0] = Base64Chars[n / 64];
            oi += 2;
        }

        return new string(result);
    }

    private static sbyte[] BuildBase16Mask()
    {
        var mask = Enumerable.Repeat((sbyte)-1, 128).ToArray();
        for (var i = 0; i < 10; i++) mask['0' + i] = (sbyte)i;
        for (var i = 0; i < 6; i++)
        {
            mask['A' + i] = (sbyte)(10 + i);
            mask['a' + i] = (sbyte)(10 + i);
        }

        return mask;
    }

    private static sbyte[] BuildBase64Mask()
    {
        var mask = Enumerable.Repeat((sbyte)-1, 128).ToArray();
        for (var i = 0; i < Base64Chars.Length; i++)
        {
            mask[Base64Chars[i]] = (sbyte)i;
        }

        return mask;
    }
}
