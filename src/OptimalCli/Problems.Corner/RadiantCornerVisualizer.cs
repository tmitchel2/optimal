/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Numerics;
using Optimal.Control.Core;
using Radiant;

namespace OptimalCli.Problems.Corner;

/// <summary>
/// Renders a real-time graphical display of the corner problem using Radiant.
/// Shows the road layout, boundaries, and vehicle trajectory through the track.
/// Converts curvilinear coordinates (s, n) to Cartesian (x, y) for display.
///
/// State format: [V, ax, ay, n, alpha, lambda, Omega, t]
///   - V: Speed (m/s)
///   - ax: Longitudinal acceleration (m/s^2)
///   - ay: Lateral acceleration (m/s^2)
///   - n: Lateral offset from centerline (m, positive = right)
///   - alpha: Heading angle relative to road (rad)
///   - lambda: Slip angle (rad)
///   - Omega: Yaw rate (rad/s)
///   - t: Elapsed time (s)
///
/// Control format: [delta, T]
///   - delta: Steering angle (rad)
///   - T: Thrust (normalized, -1 to 1)
///
/// Left-hand rule coordinate system:
///   - Positive heading = clockwise rotation
///   - heading = 0 = east, heading = +π/2 = south
///   - Direction vector: (cos(heading), -sin(heading))
/// </summary>
internal sealed class RadiantCornerVisualizer
{
    private const int WindowWidth = 900;
    private const int WindowHeight = 800;
    private static readonly Vector2 Translation = new(-300, 300);

    // State indices for the 8-state dymos model
    private const int IdxV = 0;
    private const int IdxAx = 1;
    private const int IdxAy = 2;
    private const int IdxN = 3;
    private const int IdxAlpha = 4;
    private const int IdxLambda = 5;
    private const int IdxOmega = 6;
    private const int IdxT = 7;

    // Control indices
    private const int IdxDelta = 0;
    private const int IdxThrust = 1;

    private readonly TrackGeometry _trackGeometry;

    private double[][]? _currentStates;
    private double[][]? _currentControls;
    private int _currentIteration;
    private double _currentCost;
    private double _currentMaxViolation;
    private double _currentConstraintTolerance;
    private int _currentFrameIndex;
    private DateTime _animationStartTime;
    private readonly object _lock = new();

    // Buffered next trajectory
    private double[][]? _nextStates;
    private double[][]? _nextControls;
    private int _nextIteration;
    private double _nextCost;
    private double _nextMaxViolation;
    private double _nextConstraintTolerance;
    private bool _hasNextTrajectory;

    private CancellationTokenSource? _cancellationTokenSource;

    public RadiantCornerVisualizer(TrackGeometry trackGeometry)
    {
        _trackGeometry = trackGeometry ?? throw new ArgumentNullException(nameof(trackGeometry));
    }

    public CancellationToken CancellationToken => _cancellationTokenSource?.Token ?? CancellationToken.None;

    public void UpdateTrajectory(double[][] states, double[][] controls, int iteration, double cost, double maxViolation, double constraintTolerance)
    {
        if (states.Length == 0 || controls.Length == 0)
        {
            return;
        }

        lock (_lock)
        {
            _nextStates = states;
            _nextControls = controls;
            _nextIteration = iteration;
            _nextCost = cost;
            _nextMaxViolation = maxViolation;
            _nextConstraintTolerance = constraintTolerance;
            _hasNextTrajectory = true;

            Console.WriteLine($"[VIZ] Buffered Iter {iteration}: {states.Length} frames, cost={cost:F4}, violation={maxViolation:E2}/{constraintTolerance:E2}");
        }
    }

    public void RunVisualizationWindow()
    {
        _cancellationTokenSource = new CancellationTokenSource();

        try
        {
            using var app = new RadiantApplication();
            app.Run("Racecar Minimum Time Optimization (Dymos Model)", WindowWidth, WindowHeight, renderer => RenderFrame(renderer), Colors.Slate900);
        }
        finally
        {
            if (_cancellationTokenSource?.IsCancellationRequested == false)
            {
                Console.WriteLine("[VIZ] Window closed - requesting optimization cancellation");
                _cancellationTokenSource.Cancel();
            }

            _cancellationTokenSource?.Dispose();
            _cancellationTokenSource = null;
        }
    }

    /// <summary>
    /// Run visualization in debug mode - shows track and offset lines without optimization.
    /// </summary>
    public void RunDebugVisualization(InitialGuess initialGuess)
    {
        Console.WriteLine("=== DEBUG VISUALIZATION MODE ===");
        Console.WriteLine("Showing track layout with offset lines at n = -2.5 (cyan) and n = +2.5 (orange)");
        Console.WriteLine("Close window to exit.");

        using var app = new RadiantApplication();
        app.Run("Corner Track Debug View", WindowWidth, WindowHeight, renderer => RenderDebugFrame(renderer, initialGuess), Colors.Slate900);
    }

    private void RenderDebugFrame(Radiant.Graphics2D.Renderer2D renderer, InitialGuess initialGuess)
    {
        const float scale = 15.0f;

        // Draw road layout
        DrawRoadLayout(renderer, scale);

        DrawPathTrace(renderer, initialGuess.StateTrajectory, 0, scale);

        // Draw debug info
        renderer.DrawText("DEBUG MODE - Track Layout", -400, -370, 2, Colors.Amber400);
        renderer.DrawText("Cyan line: n = -2.5 (left of centerline)", -400, -350, 2, Colors.Cyan400);
        renderer.DrawText("Orange line: n = +2.5 (right of centerline)", -400, -330, 2, Colors.Orange400);
        renderer.DrawText("Red lines: Road boundaries (n = +/-5)", -400, -310, 2, Colors.Red400);
        renderer.DrawText("White dashed: Centerline (n = 0)", -400, -290, 2, Colors.White);
    }

    private void RenderFrame(Radiant.Graphics2D.Renderer2D renderer)
    {
        double[][] states;
        double[][] controls;
        int iteration;
        double cost;
        double maxViolation;
        double constraintTolerance;
        int frameIndex;

        lock (_lock)
        {
            if (_currentStates == null || _currentControls == null)
            {
                if (_hasNextTrajectory && _nextStates != null && _nextControls != null)
                {
                    _currentStates = _nextStates;
                    _currentControls = _nextControls;
                    _currentIteration = _nextIteration;
                    _currentCost = _nextCost;
                    _currentMaxViolation = _nextMaxViolation;
                    _currentConstraintTolerance = _nextConstraintTolerance;
                    _currentFrameIndex = 0;
                    _animationStartTime = DateTime.Now;
                    _hasNextTrajectory = false;

                    Console.WriteLine($"[VIZ] Started displaying Iter {_currentIteration}");
                }
                else
                {
                    DrawRoadLayout(renderer, 15.0f);
                    renderer.DrawText("WAITING FOR OPTIMIZATION...", -200, 0, 3, Colors.Gray400);
                    return;
                }
            }

            var elapsed = (DateTime.Now - _animationStartTime).TotalMilliseconds;
            const double FrameDuration = 80.0;
            var totalFrames = _currentStates.Length;
            var frameInSequence = (int)(elapsed / FrameDuration);

            if (frameInSequence >= totalFrames && _hasNextTrajectory && _nextStates != null && _nextControls != null)
            {
                _currentStates = _nextStates;
                _currentControls = _nextControls;
                _currentIteration = _nextIteration;
                _currentCost = _nextCost;
                _currentMaxViolation = _nextMaxViolation;
                _currentConstraintTolerance = _nextConstraintTolerance;
                _currentFrameIndex = 0;
                _animationStartTime = DateTime.Now;
                _hasNextTrajectory = false;

                Console.WriteLine($"[VIZ] Switched to Iter {_currentIteration}");
                frameInSequence = 0;
                totalFrames = _currentStates.Length;
            }

            _currentFrameIndex = frameInSequence % totalFrames;

            states = _currentStates;
            controls = _currentControls;
            iteration = _currentIteration;
            cost = _currentCost;
            maxViolation = _currentMaxViolation;
            constraintTolerance = _currentConstraintTolerance;
            frameIndex = _currentFrameIndex;
        }

        // Calculate scale to fit the road in the window
        const float Scale = 15.0f;

        // Draw road layout first (background)
        DrawRoadLayout(renderer, Scale);

        // Draw trajectory path (convert curvilinear to Cartesian)
        DrawPathTrace(renderer, states, frameIndex, Scale);

        // Draw the vehicle (convert curvilinear to Cartesian)
        // State: [V, ax, ay, n, alpha, lambda, Omega, t]
        // s is computed from frame position (arc length parameterization)
        var state = states[frameIndex];
        var control = controls[frameIndex];
        var s = states.Length > 1 ? (frameIndex / (double)(states.Length - 1)) * _trackGeometry.TotalLength : 0.0;

        var velocity = state[IdxV];
        var ax = state[IdxAx];
        var ay = state[IdxAy];
        var n = state[IdxN];
        var alpha = state[IdxAlpha];  // Heading relative to road
        var elapsedTime = state[IdxT];

        var delta = control[IdxDelta];
        var thrust = control[IdxThrust];

        // Get road heading at current position
        var roadHeading = _trackGeometry.RoadHeading(s);

        // Absolute heading = road heading + alpha (relative heading)
        // Left-hand rule: positive angles are clockwise
        var absoluteHeading = roadHeading + alpha;

        // Convert curvilinear to Cartesian
        var (x, y) = _trackGeometry.CurvilinearToCartesian(s, n);

        DrawVehicle(renderer, x, y, absoluteHeading, velocity, ay, Scale);

        // Draw start and end markers
        DrawMarkers(renderer, states, Scale);

        // Draw information overlay
        DrawInformation(renderer, s, n, x, y, absoluteHeading, velocity, ax, ay, alpha, elapsedTime, delta, thrust,
            iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length);
    }

    private void DrawRoadLayout(Radiant.Graphics2D.Renderer2D renderer, float scale)
    {
        var centerLineColor = new Vector4(1.0f, 1.0f, 1.0f, 0.3f);
        var boundaryColor = new Vector4(1.0f, 0.3f, 0.3f, 0.6f);  // Red for boundaries

        // Draw dashed centerline along the entire track
        DrawDashedLine(renderer, 0.0, scale, centerLineColor);

        // Draw road boundaries at +/- RoadHalfWidth
        DrawOffsetLine(renderer, -TrackGeometry.RoadHalfWidth, scale, boundaryColor);
        DrawOffsetLine(renderer, TrackGeometry.RoadHalfWidth, scale, boundaryColor);
    }

    /// <summary>
    /// Draw a dashed line at a fixed n offset along the entire track.
    /// </summary>
    private void DrawDashedLine(Radiant.Graphics2D.Renderer2D renderer, double nOffset, float scale, Vector4 color)
    {
        const int NumSegments = 100;
        var totalLength = _trackGeometry.TotalLength;

        for (var i = 0; i < NumSegments - 1; i++)
        {
            // Only draw every other segment for dashed effect
            if (i % 2 != 0)
            {
                continue;
            }

            var s1 = totalLength * i / (NumSegments - 1);
            var s2 = totalLength * (i + 1) / (NumSegments - 1);

            var (x1, y1) = _trackGeometry.CurvilinearToCartesian(s1, nOffset);
            var (x2, y2) = _trackGeometry.CurvilinearToCartesian(s2, nOffset);

            renderer.DrawLine(
                new Vector2((float)x1 * scale, (float)y1 * scale) + Translation,
                new Vector2((float)x2 * scale, (float)y2 * scale) + Translation,
                color);
        }
    }

    /// <summary>
    /// Draw a solid line at a fixed n offset along the entire track.
    /// </summary>
    private void DrawOffsetLine(Radiant.Graphics2D.Renderer2D renderer, double nOffset, float scale, Vector4 color)
    {
        const int NumPoints = 100;
        var totalLength = _trackGeometry.TotalLength;

        for (var i = 0; i < NumPoints - 1; i++)
        {
            var s1 = totalLength * i / (NumPoints - 1);
            var s2 = totalLength * (i + 1) / (NumPoints - 1);

            var (x1, y1) = _trackGeometry.CurvilinearToCartesian(s1, nOffset);
            var (x2, y2) = _trackGeometry.CurvilinearToCartesian(s2, nOffset);

            renderer.DrawLine(
                new Vector2((float)x1 * scale, (float)y1 * scale) + Translation,
                new Vector2((float)x2 * scale, (float)y2 * scale) + Translation,
                color);
        }
    }

    private void DrawPathTrace(Radiant.Graphics2D.Renderer2D renderer, double[][] states, int currentFrame, float scale)
    {
        var totalFrames = states.Length;

        // Draw completed path
        // State: [V, ax, ay, n, alpha, lambda, Omega, t]
        // n is at index 3, V is at index 0
        for (var i = 0; i < currentFrame && i < states.Length - 1; i++)
        {
            var s1 = totalFrames > 1 ? (i / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;
            var s2 = totalFrames > 1 ? ((i + 1) / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;

            var n1 = states[i][IdxN];
            var n2 = states[i + 1][IdxN];

            var (x1, y1) = _trackGeometry.CurvilinearToCartesian(s1, n1);
            var (x2, y2) = _trackGeometry.CurvilinearToCartesian(s2, n2);

            // Color by velocity (green = slow, yellow = medium, red = fast)
            var velocity = states[i][IdxV];
            var vNorm = Math.Clamp((velocity - 10.0) / 40.0, 0.0, 1.0);  // 10-50 m/s range
            var color = new Vector4((float)vNorm, (float)(1.0 - vNorm * 0.5), 0.2f, 0.9f);

            renderer.DrawLine(new Vector2((float)x1 * scale, (float)y1 * scale) + Translation,
                             new Vector2((float)x2 * scale, (float)y2 * scale) + Translation, color);
        }

        // Draw future path in gray
        for (var i = currentFrame; i < states.Length - 1; i++)
        {
            var s1 = totalFrames > 1 ? (i / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;
            var s2 = totalFrames > 1 ? ((i + 1) / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;

            var n1 = states[i][IdxN];
            var n2 = states[i + 1][IdxN];

            var (x1, y1) = _trackGeometry.CurvilinearToCartesian(s1, n1);
            var (x2, y2) = _trackGeometry.CurvilinearToCartesian(s2, n2);

            renderer.DrawLine(new Vector2((float)x1 * scale, (float)y1 * scale) + Translation,
                             new Vector2((float)x2 * scale, (float)y2 * scale) + Translation, new Vector4(0.4f, 0.4f, 0.4f, 0.5f));
        }
    }

    private static void DrawVehicle(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double heading, double velocity, double lateralAccel, float scale)
    {
        var vehX = (float)x * scale + Translation.X;
        var vehY = (float)y * scale + Translation.Y;

        // Draw vehicle as a triangle
        // Left-hand rule: positive heading = clockwise, so heading = +π/2 = south = negative y
        // Direction vector in Cartesian: (cos(heading), -sin(heading))
        const float VehicleSize = 12.0f;
        var frontX = vehX + (float)(VehicleSize * Math.Cos(heading));
        var frontY = vehY - (float)(VehicleSize * Math.Sin(heading));

        var angle1 = heading + 2.5 * Math.PI / 3.0;
        var angle2 = heading - 2.5 * Math.PI / 3.0;

        var backLeftX = vehX + (float)(VehicleSize * 0.7 * Math.Cos(angle1));
        var backLeftY = vehY - (float)(VehicleSize * 0.7 * Math.Sin(angle1));
        var backRightX = vehX + (float)(VehicleSize * 0.7 * Math.Cos(angle2));
        var backRightY = vehY - (float)(VehicleSize * 0.7 * Math.Sin(angle2));

        // Color by velocity (green=slow, yellow=medium, red=fast)
        var vNorm = Math.Clamp((velocity - 10.0) / 40.0, 0.0, 1.0);
        var vehicleColor = new Vector4((float)vNorm, (float)(1.0 - vNorm * 0.5), 0.3f, 1.0f);

        renderer.DrawLine(new Vector2(frontX, frontY), new Vector2(backLeftX, backLeftY), vehicleColor);
        renderer.DrawLine(new Vector2(backLeftX, backLeftY), new Vector2(backRightX, backRightY), vehicleColor);
        renderer.DrawLine(new Vector2(backRightX, backRightY), new Vector2(frontX, frontY), vehicleColor);

        renderer.DrawCircleFilled(vehX, vehY, 6, vehicleColor, 16);

        // Draw lateral acceleration indicator (perpendicular to heading)
        if (Math.Abs(lateralAccel) > 0.5)
        {
            var ayScale = (float)(lateralAccel / 15.0) * 20.0f;  // Scale ay for display
            var perpAngle = heading + Math.PI / 2.0;  // Perpendicular to heading
            var ayEndX = vehX + (float)(ayScale * Math.Cos(perpAngle));
            var ayEndY = vehY - (float)(ayScale * Math.Sin(perpAngle));

            var ayColor = lateralAccel > 0 ? Colors.Orange400 : Colors.Cyan400;
            renderer.DrawLine(new Vector2(vehX, vehY), new Vector2(ayEndX, ayEndY), ayColor);
        }
    }

    private void DrawMarkers(Radiant.Graphics2D.Renderer2D renderer, double[][] states, float scale)
    {
        if (states.Length < 2) return;

        // State: [V, ax, ay, n, alpha, lambda, Omega, t]
        // n is at index 3

        // Start marker (s = 0)
        var (startX, startY) = _trackGeometry.CurvilinearToCartesian(0.0, states[0][IdxN]);
        var startPos = new Vector2((float)startX * scale, (float)startY * scale) + Translation;
        renderer.DrawCircleFilled(startPos.X, startPos.Y, 10, Colors.Emerald500, 24);
        renderer.DrawText("START", startPos.X - 30, startPos.Y + 20, 2, Colors.Emerald400);

        // End marker (s = TotalLength)
        var (endX, endY) = _trackGeometry.CurvilinearToCartesian(_trackGeometry.TotalLength, states[^1][IdxN]);
        var endPos = new Vector2((float)endX * scale, (float)endY * scale) + Translation;
        renderer.DrawCircleFilled(endPos.X, endPos.Y, 10, Colors.Rose500, 24);
        renderer.DrawText("FINISH", endPos.X - 35, endPos.Y - 15, 2, Colors.Rose400);
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer,
        double s, double n, double x, double y, double heading, double velocity,
        double ax, double ay, double alpha, double elapsedTime, double delta, double thrust,
        int iteration, double cost, double maxViolation, double constraintTolerance,
        int frameIndex, int totalFrames)
    {
        const float TopY = -370.0f;
        const float LeftX = -420.0f;
        const float RightX = 180.0f;

        // Left column - optimization status
        renderer.DrawText($"ITERATION: {iteration}", LeftX, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"LAP TIME: {cost:F3} s", LeftX, TopY + 20, 2, Colors.Sky400);

        var convergenceRatio = constraintTolerance > 0 ? maxViolation / constraintTolerance : 0.0;
        var isConverged = maxViolation < constraintTolerance;
        var convergenceColor = isConverged ? Colors.Emerald500 : (convergenceRatio < 10.0 ? Colors.Amber500 : Colors.Rose500);
        renderer.DrawText($"VIOLATION: {maxViolation:E2}", LeftX, TopY + 40, 2, convergenceColor);

        // Middle column - vehicle state
        const float MidX = -120.0f;
        renderer.DrawText($"V: {velocity:F1} m/s", MidX, TopY, 2, Colors.Cyan400);
        renderer.DrawText($"ax: {ax:F1} m/s2", MidX, TopY + 20, 2, Colors.Orange400);
        renderer.DrawText($"ay: {ay:F1} m/s2", MidX, TopY + 40, 2, Colors.Purple400);
        renderer.DrawText($"t: {elapsedTime:F2} s", MidX, TopY + 60, 2, Colors.Gray400);

        // Right column - position and control
        renderer.DrawText($"FRAME: {frameIndex + 1}/{totalFrames}", RightX, TopY, 2, Colors.Amber400);
        renderer.DrawText($"s={s:F1} n={n:F2}", RightX, TopY + 20, 2, Colors.Purple400);
        renderer.DrawText($"({x:F1}, {y:F1})", RightX, TopY + 40, 2, Colors.Purple300);

        var headingDeg = heading * 180.0 / Math.PI;
        var alphaDeg = alpha * 180.0 / Math.PI;
        renderer.DrawText($"HDG: {headingDeg:F0} DEG", RightX, TopY + 60, 2, Colors.Orange400);

        var deltaDeg = delta * 180.0 / Math.PI;
        renderer.DrawText($"STEER: {deltaDeg:F1} DEG", RightX, TopY + 80, 2, Colors.Teal400);
        renderer.DrawText($"THRUST: {thrust:F2}", RightX, TopY + 100, 2, thrust > 0 ? Colors.Green400 : Colors.Red400);
    }
}
