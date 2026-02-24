using BazanCDE.Parsing.Utilities;
using System.Text;

namespace BazanCDE.Parsing.Geometry.Operations.BooleanUtils
{
    public struct Bounds
    {
        public Vec2 min;
        public Vec2 max;

        public void Merge(Bounds other)
        {
            min = new Vec2(
                Math.Min(min.X, other.min.X),
                Math.Min(min.Y, other.min.Y));
            max = new Vec2(
                Math.Max(max.X, other.max.X),
                Math.Max(max.Y, other.max.Y));
        }
    }

    public sealed class SVGLineSet
    {
        public List<List<Vec2>> lines = new();
        public string color = "rgb(255,0,0)";

        public Bounds GetBounds(Vec2 size, Vec2 offset)
        {
            return Utils.getBounds(lines, size, offset);
        }
    }

    public sealed class SVGDrawing
    {
        public List<SVGLineSet> sets = new();

        public Bounds GetBounds(Vec2 size, Vec2 offset)
        {
            var b = new Bounds
            {
                min = new Vec2(double.MaxValue, double.MaxValue),
                max = new Vec2(-double.MaxValue, -double.MaxValue)
            };

            foreach (var set in sets)
            {
                b.Merge(set.GetBounds(size, offset));
            }

            return b;
        }
    }

    public static partial class Utils
    {
        public static Bounds getBounds(List<List<Vec2>> input, Vec2 size, Vec2 offset)
        {
            var min = new Vec2(double.MaxValue, double.MaxValue);
            var max = new Vec2(-double.MaxValue, -double.MaxValue);

            foreach (var loop in input)
            {
                foreach (var point in loop)
                {
                    min = new Vec2(
                        Math.Min(min.X, point.X),
                        Math.Min(min.Y, point.Y));
                    max = new Vec2(
                        Math.Max(max.X, point.X),
                        Math.Max(max.Y, point.Y));
                }
            }

            var width = max.X - min.X;
            var height = max.Y - min.Y;

            if (width == 0 && height == 0)
            {
                if (Eps.messages)
                {
                    Console.WriteLine("asdf");
                }
            }

            return new Bounds
            {
                min = min,
                max = max
            };
        }

        public static Vec2 rescale(Vec2 p, Bounds b, Vec2 size, Vec2 offset)
        {
            return new Vec2(
                ((p.X - b.min.X) / (b.max.X - b.min.X)) * size.X + offset.X,
                ((p.Y - b.min.Y) / (b.max.Y - b.min.Y)) * size.Y + offset.Y);
        }

        public static void svgMakeLine(Vec2 a, Vec2 b, StringBuilder svg, string col = "rgb(255,0,0)")
        {
            svg.Append("<line x1=\"").Append(a.X).Append("\" y1=\"").Append(a.Y).Append("\" ");
            svg.Append("x2=\"").Append(b.X).Append("\" y2=\"").Append(b.Y).Append("\" ");
            svg.Append("style = \"stroke:").Append(col).Append(";stroke-width:1\" />");
        }

        public static void SVGLinesToString(Bounds bounds, Vec2 size, Vec2 offset, SVGLineSet lineSet, StringBuilder svg)
        {
            foreach (var line in lineSet.lines)
            {
                if (line.Count > 1)
                {
                    for (var i = 1; i < line.Count; i++)
                    {
                        var a = rescale(line[i], bounds, size, offset);
                        var b = rescale(line[i - 1], bounds, size, offset);

                        svgMakeLine(a, b, svg, lineSet.color);
                    }
                }
                else if (line.Count == 1)
                {
                    var a = rescale(line[0], bounds, size, offset);
                    svg.Append("<circle cx = \"").Append(a.X)
                        .Append("\" cy = \"").Append(a.Y)
                        .Append("\" r = \"3\" style = \"stroke:rgb(0,0,255);stroke-width:2\" />");
                }
            }
        }

        public static string makeSVGLines(SVGDrawing drawing)
        {
            var size = new Vec2(2048, 2048);
            var offset = new Vec2(5, 5);

            var bounds = drawing.GetBounds(size, offset);

            var svg = new StringBuilder();
            svg.Append("<svg width=\"").Append(size.X + offset.X * 2)
                .Append("\" height=\"").Append(size.Y + offset.Y * 2)
                .Append("\" xmlns=\"http://www.w3.org/2000/svg\">");

            foreach (var set in drawing.sets)
            {
                SVGLinesToString(bounds, size, offset, set, svg);
            }

            svg.Append("</svg>");
            return svg.ToString();
        }

        public static void DumpSVGLines(List<List<Vec2>> lines, string filename)
        {
            var set = new SVGLineSet
            {
                lines = lines
            };

            var drawing = new SVGDrawing();
            drawing.sets.Add(set);
            File.WriteAllText(filename, makeSVGLines(drawing));
        }
    }
}
