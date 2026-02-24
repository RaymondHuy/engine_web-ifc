using BazanCDE.Parsing.Utilities;

namespace BazanCDE.Parsing.Geometry.Representation
{
    public static class IfcRepresentationConstants
    {
        public const double CONST_PI = 3.141592653589793238462643383279502884;
        public const double EPS_SMALL = 1e-6;
        public const double EPS_TINY = 1e-9;
        public const double EXTRUSION_DISTANCE_HALFSPACE_M = 100;

        public static readonly Dictionary<string, int> Horizontal_alignment_type = new(StringComparer.Ordinal)
        {
            ["LINE"] = 1,
            ["CIRCULARARC"] = 2,
            ["CLOTHOID"] = 3,
            ["CUBICSPIRAL"] = 4,
            ["BIQUADRATICPARABOLA"] = 5,
            ["BLOSSCURVE"] = 6,
            ["COSINECURVE"] = 7,
            ["SINECURVE"] = 8,
            ["VIENNESEBEND"] = 9
        };

        public static readonly Dictionary<string, int> Vertical_alignment_type = new(StringComparer.Ordinal)
        {
            ["CONSTANTGRADIENT"] = 1,
            ["CIRCULARARC"] = 2,
            ["PARABOLICARC"] = 3,
            ["CLOTHOID"] = 4
        };

        // Column-major 4x4 matrix.
        public static readonly double[] NormalizeIFC =
        {
            1, 0, 0, 0,
            0, 0, -1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1
        };
    }

    public enum IfcTrimmingSelectType
    {
        TRIM_NONE,
        TRIM_BY_POSITION,
        TRIM_BY_PARAMETER,
        TRIM_BY_LENGTH
    }

    public sealed class IfcTrimmingSelect
    {
        public IfcTrimmingSelectType trimType = IfcTrimmingSelectType.TRIM_NONE;
        public double value = 0;
        public Vec2 pos;
        public Vec3 pos3D;
    }

    public enum TrimSense
    {
        TRIM_SENSE_SAME = 1,
        TRIM_SENSE_REVERSE = 0
    }

    public sealed class IfcSegmentIndexSelect
    {
        public string type = string.Empty;
        public List<uint> indexs = new();
    }

    public sealed class IfcProfile
    {
        public string type = string.Empty;
        public IfcCurve curve = new();
        public List<IfcCurve> holes = new();
        public bool isConvex;
        public bool isComposite = false;
        public List<IfcProfile> profiles = new();
        public List<double> tags = new();
    }

    public sealed class IfcAlignmentSegment
    {
        public List<IfcCurve> curves = new();
    }

    public sealed class SweptDiskSolid
    {
        public List<IfcProfile> profiles = new();
        public List<IfcCurve> axis = new();
        public double profileRadius;
    }

    public sealed class Cylinder
    {
        public bool Active = false;
        public double Radius;
    }

    public sealed class BSpline
    {
        public bool Active = false;
        public double UDegree;
        public double VDegree;
        public string ClosedU = string.Empty;
        public string ClosedV = string.Empty;
        public string CurveType = string.Empty;
        public List<List<double>> Weights = new();
        public List<List<Vec3>> ControlPoints = new();
        public List<uint> UMultiplicity = new();
        public List<uint> VMultiplicity = new();
        public List<double> UKnots = new();
        public List<double> VKnots = new();
        public List<List<double>> WeightPoints = new();
    }

    public sealed class Revolution
    {
        public bool Active = false;
        public double[] Direction =
        {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        public IfcProfile Profile = new();
    }

    public sealed class Extrusion
    {
        public bool Active = false;
        public Vec3 Direction;
        public IfcProfile Profile = new();
        public double Length;
    }

    public sealed class IfcCrossSections
    {
        public List<IfcCurve> curves = new();
        public List<uint> expressID = new();
    }

    public sealed class IfcAlignment
    {
        public IfcAlignmentSegment Horizontal = new();
        public IfcAlignmentSegment Vertical = new();
        public IfcAlignmentSegment Absolute = new();
        public uint PlacementExpressId;

        public void transform(double[] coordinationMatrix)
        {
            if (coordinationMatrix is null || coordinationMatrix.Length < 16)
            {
                return;
            }

            uint ic = 0;
            foreach (var _ in Horizontal.curves)
            {
                if (ic > 0)
                {
                    var prev = Horizontal.curves[(int)ic - 1];
                    var cur = Horizontal.curves[(int)ic];
                    var lastId1 = prev.points.Count - 1;
                    var lastId2 = cur.points.Count - 1;

                    if (prev.points.Count > 0 && cur.points.Count > 0)
                    {
                        var d1 = (cur.points[0] - prev.points[lastId1]).Length();
                        var d2 = (cur.points[lastId2] - prev.points[lastId1]).Length();
                        if (d1 > d2)
                        {
                            cur.points.Reverse();
                        }
                    }
                }

                ic++;
            }

            ic = 0;
            foreach (var curve in Horizontal.curves)
            {
                uint ip = 0;
                foreach (var pt in curve.points)
                {
                    var p4 = RepresentationMath.MatrixMulPoint(coordinationMatrix, new Vec4(pt.X, 0, -pt.Y, 1));
                    Horizontal.curves[(int)ic].points[(int)ip] = new Vec3(
                        p4.X,
                        -p4.Z,
                        p4.Y);
                    ip++;
                }

                ic++;
            }

            ic = 0;
            foreach (var curve in Vertical.curves)
            {
                uint ip = 0;
                foreach (var pt in curve.points)
                {
                    Vertical.curves[(int)ic].points[(int)ip] = new Vec3(
                        pt.X,
                        pt.Y + coordinationMatrix[13],
                        1);
                    ip++;
                }

                ic++;
            }
        }
    }

    public sealed class IfcPlacedGeometry
    {
        public Vec4 color;
        public double[] transformation =
        {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        public double[] flatTransformation = new double[16];
        public uint geometryExpressID;

        public void SetFlatTransformation()
        {
            flatTransformation = FlattenTransformation(transformation);
        }

        public static double[] FlattenTransformation(double[] transformation)
        {
            var flat = new double[16];
            if (transformation is null || transformation.Length < 16)
            {
                return flat;
            }

            for (var i = 0; i < 4; i++)
            {
                for (var j = 0; j < 4; j++)
                {
                    flat[i * 4 + j] = transformation[i * 4 + j];
                }
            }

            return flat;
        }

        public bool testReverse()
        {
            var cx = RepresentationMath.MatrixCol3(transformation, 0);
            var cy = RepresentationMath.MatrixCol3(transformation, 1);
            var cz = RepresentationMath.MatrixCol3(transformation, 2);

            var dx = Vec3.Cross(cy, cz);
            var dy = Vec3.Cross(cx, cz);
            var dz = Vec3.Cross(cx, cy);

            var fac1 = Vec3.Dot(cx, dx);
            var fac2 = -Vec3.Dot(cy, dy);
            var fac3 = Vec3.Dot(cz, dz);

            return fac1 * fac2 * fac3 < 0;
        }
    }

    /// <summary>
    /// Represents a flattened mesh structure of an IfcComposedMesh, tracking geometries along with their respective colors and transformations.
    /// </summary>
    public sealed class IfcFlatMesh
    {
        public List<IfcPlacedGeometry> geometries = new();
        public uint expressID;
    }

    public sealed class IfcComposedMesh
    {
        public Vec4 color;
        public Mat4 transformation = Mat4.Identity;
        public uint expressID;
        public bool hasGeometry = false;
        public bool hasColor = false;
        public List<IfcComposedMesh> children = new();

        public Vec4? GetColor()
        {
            if (hasColor)
            {
                return color;
            }

            foreach (var c in children)
            {
                var col = c.GetColor();
                if (col.HasValue)
                {
                    return col;
                }
            }

            return null;
        }
    }

    public enum IfcBoundType
    {
        OUTERBOUND,
        BOUND
    }

    public sealed class IfcBound3D
    {
        public IfcBoundType type;
        public bool orientation;
        public IfcCurve curve = new();
    }

    public sealed class IfcSurface
    {
        public Mat4 transformation = Mat4.Identity;

        public BSpline BSplineSurface = new();
        public Cylinder CylinderSurface = new();
        public Revolution RevolutionSurface = new();
        public Extrusion ExtrusionSurface = new();

        public Vec3 normal()
        {
            if (!CylinderSurface.Active && !BSplineSurface.Active && !RevolutionSurface.Active)
            {
                return RepresentationMath.MatrixCol3(transformation, 2);
            }

            if (BSplineSurface.Active)
            {
                Console.WriteLine("Normal to bspline still not implemented");
            }

            if (CylinderSurface.Active)
            {
                Console.WriteLine("Normal to cylinder still not implemented");
            }

            if (RevolutionSurface.Active)
            {
                Console.WriteLine("Normal to revolution still not implemented");
            }

            return new Vec3(0, 0, 0);
        }
    }

    internal static class RepresentationMath
    {
        internal static Vec4 MatrixMulPoint(double[] m, Vec4 v)
        {
            var x = m[0] * v.X + m[4] * v.Y + m[8] * v.Z + m[12] * v.W;
            var y = m[1] * v.X + m[5] * v.Y + m[9] * v.Z + m[13] * v.W;
            var z = m[2] * v.X + m[6] * v.Y + m[10] * v.Z + m[14] * v.W;
            var w = m[3] * v.X + m[7] * v.Y + m[11] * v.Z + m[15] * v.W;
            return new Vec4(x, y, z, w);
        }

        internal static Vec3 MatrixCol3(double[] m, int col)
        {
            var o = col * 4;
            return new Vec3(m[o + 0], m[o + 1], m[o + 2]);
        }
    }

}
