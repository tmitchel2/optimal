/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace OptimalCli.Problems.Corner;

/// <summary>
/// Represents a single segment of track geometry (either a line or arc).
/// </summary>
public abstract class TrackSegment
{
    /// <summary>Start position along the centerline (s coordinate).</summary>
    public double StartS { get; init; }

    /// <summary>Length of this segment along the centerline.</summary>
    public abstract double Length { get; }

    /// <summary>End position along the centerline.</summary>
    public double EndS => StartS + Length;

    /// <summary>Road heading at the start of this segment.</summary>
    public double StartHeading { get; init; }

    /// <summary>Road heading at the end of this segment.</summary>
    public abstract double EndHeading { get; }

    /// <summary>Cartesian start position.</summary>
    public (double X, double Y) StartPosition { get; init; }

    /// <summary>Cartesian end position.</summary>
    public abstract (double X, double Y) EndPosition { get; }

    /// <summary>Get road heading at position s within this segment.</summary>
    public abstract double GetHeading(double s);

    /// <summary>Get road curvature at position s within this segment.</summary>
    public abstract double GetCurvature(double s);

    /// <summary>Convert curvilinear (s, n) to Cartesian (x, y) within this segment.</summary>
    public abstract (double X, double Y) ToCartesian(double s, double n);
}

/// <summary>
/// Represents a straight line segment.
/// </summary>
public sealed class LineSegment : TrackSegment
{
    private readonly double _length;

    public LineSegment(double startS, double length, double heading, (double X, double Y) startPosition)
    {
        StartS = startS;
        _length = length;
        StartHeading = heading;
        StartPosition = startPosition;
    }

    public override double Length => _length;

    public override double EndHeading => StartHeading;

    public override (double X, double Y) EndPosition
    {
        get
        {
            // Left-hand rule: heading 0 = east, heading +π/2 = south
            // Direction vector: (cos(heading), -sin(heading))
            var x = StartPosition.X + _length * Math.Cos(StartHeading);
            var y = StartPosition.Y - _length * Math.Sin(StartHeading);
            return (x, y);
        }
    }

    public override double GetHeading(double s) => StartHeading;

    public override double GetCurvature(double s) => 0.0;

    public override (double X, double Y) ToCartesian(double s, double n)
    {
        var progress = s - StartS;
        // Centerline position
        var cx = StartPosition.X + progress * Math.Cos(StartHeading);
        var cy = StartPosition.Y - progress * Math.Sin(StartHeading);
        // Perpendicular offset (n positive = right of centerline)
        // For left-hand rule: forward = (cos θ, -sin θ), right = (-sin θ, -cos θ)
        // So offset = n * (-sin θ, -cos θ)
        return (cx - n * Math.Sin(StartHeading), cy - n * Math.Cos(StartHeading));
    }
}

/// <summary>
/// Represents a circular arc segment.
/// </summary>
public sealed class ArcSegment : TrackSegment
{
    private readonly double _radius;
    private readonly double _sweepAngle; // Positive for right turn (left-hand rule)

    public ArcSegment(double startS, double radius, double sweepAngle,
                     double startHeading, (double X, double Y) startPosition)
    {
        StartS = startS;
        _radius = radius;
        _sweepAngle = sweepAngle;
        StartHeading = startHeading;
        StartPosition = startPosition;

        // Compute arc center
        // For left-hand rule: center is perpendicular to heading
        // Right turn (positive sweep): center is to the right of path (below in y for heading=0)
        // The perpendicular direction pointing right is (sin(heading), -cos(heading))
        var sign = Math.Sign(_sweepAngle);
        Center = (
            startPosition.X + sign * _radius * Math.Sin(startHeading),
            startPosition.Y - sign * _radius * Math.Cos(startHeading)
        );
    }

    public double Radius => _radius;
    public double SweepAngle => _sweepAngle;
    public (double X, double Y) Center { get; }

    public override double Length => Math.Abs(_sweepAngle) * _radius;

    public override double EndHeading => StartHeading + _sweepAngle;

    public override (double X, double Y) EndPosition
    {
        get
        {
            var endHeading = EndHeading;
            var sign = Math.Sign(_sweepAngle);
            // Position on circle: center + sign * radius * (sin(heading), cos(heading))
            // This follows from the geometric relationship between road heading and position
            return (
                Center.X + sign * _radius * Math.Sin(endHeading),
                Center.Y + sign * _radius * Math.Cos(endHeading)
            );
        }
    }

    public override double GetHeading(double s)
    {
        var progress = (s - StartS) / Length;
        return StartHeading + progress * _sweepAngle;
    }

    public override double GetCurvature(double s)
    {
        // Curvature = 1/R with sign for turn direction
        return _sweepAngle > 0 ? 1.0 / _radius : -1.0 / _radius;
    }

    public override (double X, double Y) ToCartesian(double s, double n)
    {
        var progress = (s - StartS) / Length;
        var heading = StartHeading + progress * _sweepAngle;
        var sign = Math.Sign(_sweepAngle);

        // Centerline position on arc: center + sign * radius * (sin(heading), cos(heading))
        var cx = Center.X + sign * _radius * Math.Sin(heading);
        var cy = Center.Y + sign * _radius * Math.Cos(heading);

        // Perpendicular offset (n positive = right of centerline)
        // For left-hand rule: forward = (cos θ, -sin θ), right = (-sin θ, -cos θ)
        // So offset = n * (-sin θ, -cos θ)
        return (cx - n * Math.Sin(heading), cy - n * Math.Cos(heading));
    }
}

/// <summary>
/// Immutable track geometry built from segments.
/// Provides efficient O(log n) lookup by s position using binary search.
/// </summary>
public sealed class TrackGeometry
{
    /// <summary>
    /// Half-width of the road (distance from centerline to edge).
    /// This is a constraint parameter, not a geometric property of the track centerline.
    /// </summary>
    public const double RoadHalfWidth = 5.0;

    private readonly TrackSegment[] _segments;
    private readonly double[] _segmentEndS; // Cached for binary search

    internal TrackGeometry(TrackSegment[] segments)
    {
        _segments = segments;
        _segmentEndS = new double[segments.Length];
        for (var i = 0; i < segments.Length; i++)
        {
            _segmentEndS[i] = segments[i].EndS;
        }
    }

    /// <summary>Total length of the track.</summary>
    public double TotalLength => _segments.Length > 0 ? _segments[^1].EndS : 0.0;

    /// <summary>Number of segments.</summary>
    public int SegmentCount => _segments.Length;

    /// <summary>Get segment by index.</summary>
    public TrackSegment this[int index] => _segments[index];

    /// <summary>
    /// Find the segment containing position s using binary search.
    /// O(log n) complexity.
    /// </summary>
    public TrackSegment GetSegmentAt(double s)
    {
        if (_segments.Length == 0)
        {
            throw new InvalidOperationException("No segments defined.");
        }

        // Clamp s to valid range
        s = Math.Clamp(s, 0, TotalLength);

        // Binary search for the segment
        var index = Array.BinarySearch(_segmentEndS, s);
        if (index < 0)
        {
            index = ~index; // Get insertion point
        }

        return _segments[Math.Min(index, _segments.Length - 1)];
    }

    /// <summary>
    /// Get road heading at position s along centerline.
    /// </summary>
    public double RoadHeading(double s)
    {
        return GetSegmentAt(s).GetHeading(s);
    }

    /// <summary>
    /// Get road curvature at position s along centerline.
    /// </summary>
    public double RoadCurvature(double s)
    {
        return GetSegmentAt(s).GetCurvature(s);
    }

    /// <summary>
    /// Convert curvilinear coordinates (s, n) to Cartesian (x, y).
    /// </summary>
    public (double X, double Y) CurvilinearToCartesian(double s, double n)
    {
        return GetSegmentAt(s).ToCartesian(s, n);
    }

    /// <summary>
    /// Get the length of the first line segment (entry length).
    /// </summary>
    public double GetEntryLength()
    {
        if (_segments.Length > 0 && _segments[0] is LineSegment line)
        {
            return line.Length;
        }
        return 0.0;
    }

    /// <summary>
    /// Get the arc length of the first arc segment.
    /// </summary>
    public double GetArcLength()
    {
        foreach (var segment in _segments)
        {
            if (segment is ArcSegment arc)
            {
                return arc.Length;
            }
        }
        return 0.0;
    }

    /// <summary>
    /// Get the radius of the first arc segment.
    /// </summary>
    public double GetArcRadius()
    {
        foreach (var segment in _segments)
        {
            if (segment is ArcSegment arc)
            {
                return arc.Radius;
            }
        }
        return 0.0;
    }

    // === FLUENT BUILDER ===

    /// <summary>
    /// Start building a track geometry at the specified position and heading.
    /// </summary>
    /// <param name="x">Starting X coordinate (Cartesian).</param>
    /// <param name="y">Starting Y coordinate (Cartesian).</param>
    /// <param name="heading">Starting heading (0 = east, +π/2 = south in left-hand rule).</param>
    public static TrackGeometryBuilder StartAt(double x, double y, double heading)
    {
        return new TrackGeometryBuilder(x, y, heading);
    }
}

/// <summary>
/// Fluent builder for constructing TrackGeometry.
/// </summary>
public sealed class TrackGeometryBuilder
{
    private readonly List<TrackSegment> _segments = [];
    private double _currentS;
    private double _currentHeading;
    private (double X, double Y) _currentPosition;

    internal TrackGeometryBuilder(double x, double y, double heading)
    {
        _currentPosition = (x, y);
        _currentHeading = heading;
        _currentS = 0.0;
    }

    /// <summary>
    /// Add a straight line segment.
    /// </summary>
    /// <param name="distance">Length of the line segment.</param>
    public TrackGeometryBuilder AddLine(double distance)
    {
        if (distance <= 0)
        {
            throw new ArgumentException("Distance must be positive.", nameof(distance));
        }

        var segment = new LineSegment(_currentS, distance, _currentHeading, _currentPosition);
        _segments.Add(segment);

        _currentS = segment.EndS;
        _currentPosition = segment.EndPosition;
        // Heading unchanged for straight line

        return this;
    }

    /// <summary>
    /// Add a circular arc segment.
    /// </summary>
    /// <param name="radius">Radius of the arc.</param>
    /// <param name="angle">Sweep angle in radians (absolute value).</param>
    /// <param name="turnRight">True for right turn (positive curvature in left-hand rule).</param>
    public TrackGeometryBuilder AddArc(double radius, double angle, bool turnRight = true)
    {
        if (radius <= 0)
        {
            throw new ArgumentException("Radius must be positive.", nameof(radius));
        }
        if (angle <= 0)
        {
            throw new ArgumentException("Angle must be positive.", nameof(angle));
        }

        var sweepAngle = turnRight ? angle : -angle;
        var segment = new ArcSegment(_currentS, radius, sweepAngle, _currentHeading, _currentPosition);
        _segments.Add(segment);

        _currentS = segment.EndS;
        _currentPosition = segment.EndPosition;
        _currentHeading = segment.EndHeading;

        return this;
    }

    /// <summary>
    /// Build the final immutable TrackGeometry.
    /// </summary>
    public TrackGeometry Build()
    {
        if (_segments.Count == 0)
        {
            throw new InvalidOperationException("Cannot build empty track geometry.");
        }

        return new TrackGeometry([.. _segments]);
    }
}
