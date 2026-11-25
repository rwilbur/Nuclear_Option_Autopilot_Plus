using System.Collections.Generic;
using System.Globalization;
using System.Text;

/// <summary>Minimal JSON reader for the airframe exports: objects -> Dictionary, arrays -> List, numbers -> double.</summary>
internal static class MiniJson
{
    public static object Parse(string s)
    {
        int i = 0;
        return Value(s, ref i);
    }

    private static object Value(string s, ref int i)
    {
        Skip(s, ref i);
        char c = s[i];
        if (c == '{')
        {
            var d = new Dictionary<string, object>();
            i++;
            Skip(s, ref i);
            if (s[i] == '}') { i++; return d; }
            while (true)
            {
                Skip(s, ref i);
                string k = Str(s, ref i);
                Skip(s, ref i);
                i++; // ':'
                d[k] = Value(s, ref i);
                Skip(s, ref i);
                if (s[i++] == '}') return d;
            }
        }
        if (c == '[')
        {
            var l = new List<object>();
            i++;
            Skip(s, ref i);
            if (s[i] == ']') { i++; return l; }
            while (true)
            {
                l.Add(Value(s, ref i));
                Skip(s, ref i);
                if (s[i++] == ']') return l;
            }
        }
        if (c == '"') return Str(s, ref i);
        if (string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return true; }
        if (string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return false; }
        if (string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
        int start = i;
        while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
        return double.Parse(s.Substring(start, i - start), CultureInfo.InvariantCulture);
    }

    private static string Str(string s, ref int i)
    {
        var sb = new StringBuilder();
        i++; // opening quote
        while (s[i] != '"')
        {
            if (s[i] == '\\')
            {
                char e = s[++i];
                if (e == 'u') { sb.Append((char)int.Parse(s.Substring(i + 1, 4), NumberStyles.HexNumber)); i += 4; }
                else sb.Append(e == 'n' ? '\n' : e == 't' ? '\t' : e);
            }
            else sb.Append(s[i]);
            i++;
        }
        i++;
        return sb.ToString();
    }

    private static void Skip(string s, ref int i)
    {
        while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
    }
}
