using System.Numerics;

namespace BazanCDE.Parsing.Operations;

public sealed class Arc : Curve
{
    public double RadiusX { get; private set; }
    public double RadiusY { get; private set; }
    public int NumSegments { get; private set; } = 16;
    public double[] Placement { get; private set; } =
    [
        1d, 0d, 0d,
        0d, 1d, 0d,
        0d, 0d, 1d
    ];

    public double StartRad { get; private set; }
    public double EndRad { get; private set; } = GeometryOps.ConstPi * 2;
    public bool Swap { get; private set; } = true;
    public bool NormalToCenterEnding { get; private set; }

    public void SetValues(double radiusX, double radiusY, int numSegments, IReadOnlyList<double> placement, double startRad = 0, double endRad = GeometryOps.ConstPi * 2, bool swap = true, bool normalToCenterEnding = false)
    {
        RadiusX = radiusX;
        RadiusY = radiusY;
        NumSegments = Math.Max(3, numSegments);
        StartRad = startRad;
        EndRad = endRad;
        Swap = swap;
        NormalToCenterEnding = normalToCenterEnding;
        Placement = placement is { Count: 9 } ? placement.ToArray() :
        [
            1d, 0d, 0d,
            0d, 1d, 0d,
            0d, 0d, 1d
        ];
    }

    public Buffers GetBuffers()
    {
        Points.Clear();
        var curve = GeometryOps.GetEllipseCurve(RadiusX, RadiusY, NumSegments, Placement, StartRad, EndRad, Swap, NormalToCenterEnding);
        Points.AddRange(curve.Points);

        var buffers = new Buffers();
        foreach (var p in Points)
        {
            buffers.AddPoint(p);
        }

        return buffers;
    }
}

public sealed class Parabola : Curve
{
    public int Segments { get; private set; } = 16;
    public Vector2 StartPoint { get; private set; }
    public double HorizontalLength { get; private set; }
    public double StartHeight { get; private set; }
    public double StartGradient { get; private set; }
    public double EndGradient { get; private set; }

    public void SetValues(int segments, double startPointX, double startPointY, double startPointZ, double horizontalLength, double startHeight, double startGradient, double endGradient)
    {
        _ = startPointZ;
        Segments = Math.Max(2, segments);
        StartPoint = new Vector2((float)startPointX, (float)startPointY);
        HorizontalLength = horizontalLength;
        StartHeight = startHeight;
        StartGradient = startGradient;
        EndGradient = endGradient;
    }

    public Buffers GetBuffers()
    {
        Points.Clear();
        var p2 = GeometryOps.SolveParabola(Segments, StartPoint, HorizontalLength, StartHeight, StartGradient, EndGradient);
        foreach (var p in p2)
        {
            Points.Add(new Vector3(p, 0));
        }

        var buffers = new Buffers();
        foreach (var p in Points)
        {
            buffers.AddPoint(p);
        }

        return buffers;
    }
}

public sealed class Clothoid : Curve
{
    public int Segments { get; private set; } = 16;
    public Vector2 StartPoint { get; private set; }
    public double IfcStartDirection { get; private set; }
    public double StartRadiusOfCurvature { get; private set; }
    public double EndRadiusOfCurvature { get; private set; }
    public double SegmentLength { get; private set; }

    public void SetValues(int segments, double startPointX, double startPointY, double startPointZ, double ifcStartDirection, double startRadiusOfCurvature, double endRadiusOfCurvature, double segmentLength)
    {
        _ = startPointZ;
        Segments = Math.Max(4, segments);
        StartPoint = new Vector2((float)startPointX, (float)startPointY);
        IfcStartDirection = ifcStartDirection;
        StartRadiusOfCurvature = startRadiusOfCurvature;
        EndRadiusOfCurvature = endRadiusOfCurvature;
        SegmentLength = segmentLength;
    }

    public Buffers GetBuffers()
    {
        Points.Clear();
        var p2 = GeometryOps.SolveClothoid(Segments, StartPoint, IfcStartDirection, StartRadiusOfCurvature, EndRadiusOfCurvature, SegmentLength);
        foreach (var p in p2)
        {
            Points.Add(new Vector3(p, 0));
        }

        var buffers = new Buffers();
        foreach (var p in Points)
        {
            buffers.AddPoint(p);
        }

        return buffers;
    }
}

public sealed class Alignment : Curve
{
    public List<Vector3> Horizontal { get; } = new();
    public List<Vector3> Vertical { get; } = new();

    public void SetValues(IReadOnlyList<double> horizontal, IReadOnlyList<double> vertical)
    {
        Horizontal.Clear();
        Vertical.Clear();

        for (var i = 0; i + 2 < horizontal.Count; i += 3)
        {
            Horizontal.Add(new Vector3((float)horizontal[i], (float)horizontal[i + 1], (float)horizontal[i + 2]));
        }

        for (var i = 0; i + 2 < vertical.Count; i += 3)
        {
            Vertical.Add(new Vector3((float)vertical[i], (float)vertical[i + 1], (float)vertical[i + 2]));
        }
    }

    public Buffers GetBuffers()
    {
        Points.Clear();
        Points.AddRange(GeometryOps.Convert2DAlignmentsTo3D(Horizontal, Vertical));

        var buffers = new Buffers();
        foreach (var p in Points)
        {
            buffers.AddPoint(p);
        }

        return buffers;
    }
}

public sealed class Profile
{
    public ushort ProfileType { get; private set; }
    public double Width { get; private set; }
    public double Depth { get; private set; }
    public double Thickness { get; private set; }
    public double FlangeThickness { get; private set; }
    public bool HasFillet { get; private set; }
    public double FilletRadius { get; private set; }
    public double Radius { get; private set; }
    public double Slope { get; private set; }
    public ushort NumSegments { get; private set; }
    public List<double> Placement { get; } = new();

    public Curve ProfileCurve { get; private set; } = new();

    public void SetValues(ushort profileType, double width, double depth, double webThickness, double flangeThickness, bool hasFillet, double filletRadius, double radius, double slope, ushort numSegments, IReadOnlyList<double> placement)
    {
        ProfileType = profileType;
        Width = width;
        Depth = depth;
        Thickness = webThickness;
        FlangeThickness = flangeThickness;
        HasFillet = hasFillet;
        FilletRadius = filletRadius;
        Radius = radius;
        Slope = slope;
        NumSegments = numSegments;

        Placement.Clear();
        Placement.AddRange(placement);
    }

    public Buffers GetBuffers()
    {
        var matrix = Placement.Count == 16
            ? new Matrix4x4(
                (float)Placement[0], (float)Placement[1], (float)Placement[2], (float)Placement[3],
                (float)Placement[4], (float)Placement[5], (float)Placement[6], (float)Placement[7],
                (float)Placement[8], (float)Placement[9], (float)Placement[10], (float)Placement[11],
                (float)Placement[12], (float)Placement[13], (float)Placement[14], (float)Placement[15])
            : Matrix4x4.Identity;

        ProfileCurve = ProfileType switch
        {
            0 => GeometryOps.GetIShapedCurve(Width, Depth, Thickness, FlangeThickness, HasFillet, FilletRadius, matrix),
            1 => GeometryOps.GetCShapedCurve(Width, Depth, Thickness, FlangeThickness, HasFillet, FilletRadius, matrix),
            2 => GeometryOps.GetZShapedCurve(Width, Depth, Thickness, FlangeThickness, HasFillet, FilletRadius, matrix),
            3 => GeometryOps.GetTShapedCurve(Width, Depth, Thickness, HasFillet, FilletRadius, Radius, Slope, matrix),
            4 => GeometryOps.GetLShapedCurve(Width, Depth, Thickness, HasFillet, FilletRadius, Radius, Slope, NumSegments, matrix),
            5 => GeometryOps.GetUShapedCurve(Width, Depth, Thickness, FlangeThickness, FilletRadius, Radius, Slope, matrix),
            _ => new Curve()
        };

        var buffers = new Buffers();
        foreach (var p in ProfileCurve.Points)
        {
            buffers.AddPoint(p);
        }

        return buffers;
    }
}

public sealed class Extrusion
{
    public bool Cap { get; private set; } = true;
    public double Length { get; private set; }
    public Vector3 Direction { get; private set; } = Vector3.UnitZ;
    public Vector3 CuttingPlanePosition { get; private set; }
    public Vector3 CuttingPlaneNormal { get; private set; } = Vector3.UnitZ;

    public List<Vector3> Profile { get; } = new();
    public List<List<Vector3>> Holes { get; } = new();

    public void SetValues(IReadOnlyList<double> profile, IReadOnlyList<double> dir, double len, IReadOnlyList<double> cuttingPlaneNormal, IReadOnlyList<double> cuttingPlanePos, bool cap)
    {
        Profile.Clear();
        for (var i = 0; i + 2 < profile.Count; i += 3)
        {
            Profile.Add(new Vector3((float)profile[i], (float)profile[i + 1], (float)profile[i + 2]));
        }

        if (dir.Count >= 3)
        {
            Direction = new Vector3((float)dir[0], (float)dir[1], (float)dir[2]);
        }

        Length = len;
        Cap = cap;

        if (cuttingPlaneNormal.Count >= 3)
        {
            CuttingPlaneNormal = new Vector3((float)cuttingPlaneNormal[0], (float)cuttingPlaneNormal[1], (float)cuttingPlaneNormal[2]);
        }

        if (cuttingPlanePos.Count >= 3)
        {
            CuttingPlanePosition = new Vector3((float)cuttingPlanePos[0], (float)cuttingPlanePos[1], (float)cuttingPlanePos[2]);
        }
    }

    public void SetHoles(IReadOnlyList<double> hole)
    {
        var points = new List<Vector3>();
        for (var i = 0; i + 2 < hole.Count; i += 3)
        {
            points.Add(new Vector3((float)hole[i], (float)hole[i + 1], (float)hole[i + 2]));
        }

        Holes.Add(points);
    }

    public void ClearHoles()
    {
        Holes.Clear();
    }

    public Buffers GetBuffers()
    {
        Geometry geom;
        if (!Cap)
        {
            geom = GeometryOps.Extrude(Profile, Direction, Length, cap: false);
        }
        else
        {
            var allProfiles = new List<IReadOnlyList<Vector3>> { Profile };
            allProfiles.AddRange(Holes);
            geom = GeometryOps.Extrude(allProfiles, Direction, Length, CuttingPlaneNormal, CuttingPlanePosition);
        }

        var buffers = new Buffers();
        for (var i = 0u; i < geom.NumFaces; i++)
        {
            var f = geom.GetFace(i);
            buffers.AddTri(
                geom.GetPoint((uint)f.I0),
                geom.GetPoint((uint)f.I1),
                geom.GetPoint((uint)f.I2));
        }

        return buffers;
    }
}

public sealed class Revolve
{
    public double NumRots { get; private set; } = 12;
    public Matrix4x4 Transform { get; private set; } = Matrix4x4.Identity;
    public double StartDegrees { get; private set; }
    public double EndDegrees { get; private set; } = 360;
    public List<Vector3> Profile { get; } = new();

    public void SetValues(IReadOnlyList<double> profile, IReadOnlyList<double> transform, double startDegrees, double endDegrees, uint numRots)
    {
        NumRots = Math.Max(2, numRots);
        Profile.Clear();
        for (var i = 0; i + 2 < profile.Count; i += 3)
        {
            Profile.Add(new Vector3((float)profile[i], (float)profile[i + 1], (float)profile[i + 2]));
        }

        if (transform.Count == 16)
        {
            Transform = new Matrix4x4(
                (float)transform[0], (float)transform[1], (float)transform[2], (float)transform[3],
                (float)transform[4], (float)transform[5], (float)transform[6], (float)transform[7],
                (float)transform[8], (float)transform[9], (float)transform[10], (float)transform[11],
                (float)transform[12], (float)transform[13], (float)transform[14], (float)transform[15]);
        }

        StartDegrees = startDegrees;
        EndDegrees = endDegrees;
    }

    public Buffers GetBuffers()
    {
        var geom = GeometryOps.Revolution(Transform, StartDegrees, EndDegrees, Profile, NumRots);
        var buffers = new Buffers();

        for (var i = 0u; i < geom.NumFaces; i++)
        {
            var f = geom.GetFace(i);
            buffers.AddTri(geom.GetPoint((uint)f.I0), geom.GetPoint((uint)f.I1), geom.GetPoint((uint)f.I2));
        }

        return buffers;
    }
}

public sealed class CylindricalRevolution
{
    public double NumRots { get; private set; } = 12;
    public Matrix4x4 Transform { get; private set; } = Matrix4x4.Identity;
    public double StartDegrees { get; private set; }
    public double EndDegrees { get; private set; } = 360;
    public double MinZ { get; private set; }
    public double MaxZ { get; private set; }
    public double Radius { get; private set; }

    public void SetValues(IReadOnlyList<double> transform, double startDegrees, double endDegrees, double minZ, double maxZ, double numRots, double radius)
    {
        if (transform.Count == 16)
        {
            Transform = new Matrix4x4(
                (float)transform[0], (float)transform[1], (float)transform[2], (float)transform[3],
                (float)transform[4], (float)transform[5], (float)transform[6], (float)transform[7],
                (float)transform[8], (float)transform[9], (float)transform[10], (float)transform[11],
                (float)transform[12], (float)transform[13], (float)transform[14], (float)transform[15]);
        }

        StartDegrees = startDegrees;
        EndDegrees = endDegrees;
        MinZ = minZ;
        MaxZ = maxZ;
        Radius = radius;
        NumRots = Math.Max(2, numRots);
    }

    public Buffers GetBuffers()
    {
        var geom = GeometryOps.RevolveCylinder(Transform, StartDegrees, EndDegrees, MinZ, MaxZ, (int)NumRots, Radius);
        var buffers = new Buffers();

        for (var i = 0u; i < geom.NumFaces; i++)
        {
            var f = geom.GetFace(i);
            buffers.AddTri(geom.GetPoint((uint)f.I0), geom.GetPoint((uint)f.I1), geom.GetPoint((uint)f.I2));
        }

        return buffers;
    }
}

public sealed class Sweep
{
    public double Scaling { get; private set; } = 1;
    public bool Closed { get; private set; }
    public List<Vector3> ProfilePoints { get; } = new();
    public List<Vector3> Directrix { get; } = new();
    public Vector3 InitialDirectrixNormal { get; private set; } = Vector3.UnitY;
    public bool Rotate90 { get; private set; }
    public bool Optimize { get; private set; }

    public void SetValues(double scaling, bool closed, IReadOnlyList<double> profilePoints, IReadOnlyList<double> directrix, IReadOnlyList<double> initialDirectrixNormal, bool rotate90, bool optimize)
    {
        Scaling = scaling;
        Closed = closed;
        Rotate90 = rotate90;
        Optimize = optimize;

        ProfilePoints.Clear();
        for (var i = 0; i + 2 < profilePoints.Count; i += 3)
        {
            ProfilePoints.Add(new Vector3((float)profilePoints[i], (float)profilePoints[i + 1], (float)profilePoints[i + 2]));
        }

        Directrix.Clear();
        for (var i = 0; i + 2 < directrix.Count; i += 3)
        {
            Directrix.Add(new Vector3((float)directrix[i], (float)directrix[i + 1], (float)directrix[i + 2]));
        }

        if (initialDirectrixNormal.Count >= 3)
        {
            InitialDirectrixNormal = new Vector3((float)initialDirectrixNormal[0], (float)initialDirectrixNormal[1], (float)initialDirectrixNormal[2]);
        }
    }

    public Buffers GetBuffers()
    {
        var geom = GeometryOps.SweepFunction(Scaling, Closed, ProfilePoints, Directrix, InitialDirectrixNormal, Rotate90, Optimize);
        var buffers = new Buffers();

        for (var i = 0u; i < geom.NumFaces; i++)
        {
            var f = geom.GetFace(i);
            buffers.AddTri(geom.GetPoint((uint)f.I0), geom.GetPoint((uint)f.I1), geom.GetPoint((uint)f.I2));
        }

        return buffers;
    }
}

public sealed class CircularSweep
{
    public double Scaling { get; private set; } = 1;
    public bool Closed { get; private set; }
    public List<Vector3> ProfilePoints { get; } = new();
    public double Radius { get; private set; } = 1;
    public List<Vector3> Directrix { get; } = new();
    public Vector3 InitialDirectrixNormal { get; private set; } = Vector3.UnitY;
    public bool Rotate90 { get; private set; }

    public void SetValues(double scaling, bool closed, IReadOnlyList<double> profilePoints, double radius, IReadOnlyList<double> directrix, IReadOnlyList<double> initialDirectrixNormal, bool rotate90)
    {
        Scaling = scaling;
        Closed = closed;
        Radius = radius;
        Rotate90 = rotate90;

        ProfilePoints.Clear();
        for (var i = 0; i + 2 < profilePoints.Count; i += 3)
        {
            ProfilePoints.Add(new Vector3((float)profilePoints[i], (float)profilePoints[i + 1], (float)profilePoints[i + 2]));
        }

        Directrix.Clear();
        for (var i = 0; i + 2 < directrix.Count; i += 3)
        {
            Directrix.Add(new Vector3((float)directrix[i], (float)directrix[i + 1], (float)directrix[i + 2]));
        }

        if (initialDirectrixNormal.Count >= 3)
        {
            InitialDirectrixNormal = new Vector3((float)initialDirectrixNormal[0], (float)initialDirectrixNormal[1], (float)initialDirectrixNormal[2]);
        }
    }

    public Buffers GetBuffers()
    {
        var geom = GeometryOps.SweepCircular(Scaling, Closed, ProfilePoints, Radius, Directrix, InitialDirectrixNormal, Rotate90);
        var buffers = new Buffers();

        for (var i = 0u; i < geom.NumFaces; i++)
        {
            var f = geom.GetFace(i);
            buffers.AddTri(geom.GetPoint((uint)f.I0), geom.GetPoint((uint)f.I1), geom.GetPoint((uint)f.I2));
        }

        return buffers;
    }
}

public sealed class Boolean
{
    public string Op { get; private set; } = string.Empty;
    public Geometry Geometry { get; private set; } = new();
    public List<Geometry> Seconds { get; } = new();

    public void SetValues(IReadOnlyList<double> triangles, string op)
    {
        Op = op;
        Geometry = new Geometry();

        for (var i = 0; i + 8 < triangles.Count; i += 9)
        {
            Geometry.AddFace(
                new Vector3((float)triangles[i], (float)triangles[i + 1], (float)triangles[i + 2]),
                new Vector3((float)triangles[i + 3], (float)triangles[i + 4], (float)triangles[i + 5]),
                new Vector3((float)triangles[i + 6], (float)triangles[i + 7], (float)triangles[i + 8]));
        }

        Geometry.BuildPlanes();
    }

    public void SetSecond(IReadOnlyList<double> triangles)
    {
        var g = new Geometry();
        for (var i = 0; i + 8 < triangles.Count; i += 9)
        {
            g.AddFace(
                new Vector3((float)triangles[i], (float)triangles[i + 1], (float)triangles[i + 2]),
                new Vector3((float)triangles[i + 3], (float)triangles[i + 4], (float)triangles[i + 5]),
                new Vector3((float)triangles[i + 6], (float)triangles[i + 7], (float)triangles[i + 8]));
        }

        g.BuildPlanes();
        Seconds.Add(g);
    }

    public void Clear()
    {
        Geometry = new Geometry();
        Seconds.Clear();
        Op = string.Empty;
    }

    public Buffers GetBuffers()
    {
        var geom = GeometryOps.BoolProcess(Geometry, Seconds, Op);
        var buffers = new Buffers();
        for (var i = 0u; i < geom.NumFaces; i++)
        {
            var f = geom.GetFace(i);
            buffers.AddTri(geom.GetPoint((uint)f.I0), geom.GetPoint((uint)f.I1), geom.GetPoint((uint)f.I2));
        }

        return buffers;
    }
}
