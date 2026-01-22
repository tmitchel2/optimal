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
/// Shows the road layout, boundaries, and vehicle trajectory through the 90° turn.
/// Converts curvilinear coordinates (s, n) to Cartesian (x, y) for display.
/// </summary>
internal sealed class RadiantCornerVisualizer
{
    private const int WindowWidth = 900;
    private const int WindowHeight = 800;
    private static readonly Vector2 Translation = new(-300, 300);

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
            app.Run("Corner Racing Line Optimization (Curvilinear)", WindowWidth, WindowHeight, renderer => RenderFrame(renderer), Colors.Slate900);
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
        renderer.DrawText("Red lines: Road boundaries (n = ±5)", -400, -310, 2, Colors.Red400);
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
        // s is derived from frame index: s = (frameIndex / (totalFrames - 1)) * _trackGeometry.TotalLength
        DrawPathTrace(renderer, states, frameIndex, Scale);

        // Draw the vehicle (convert curvilinear to Cartesian)
        // State is now [n, θ, v, T_f] - s is computed from frame position
        var state = states[frameIndex];
        var s = states.Length > 1 ? (frameIndex / (double)(states.Length - 1)) * _trackGeometry.TotalLength : 0.0;
        var n = state[0];
        var theta = state[1];
        var v = state[2];
        var (x, y) = _trackGeometry.CurvilinearToCartesian(s, n);
        DrawVehicle(renderer, x, y, theta, v, Scale);

        // Draw start and end markers
        DrawMarkers(renderer, states, Scale);

        // Draw information overlay
        DrawInformation(renderer, s, n, x, y, theta, v,
            iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length);
    }

    private void DrawRoadLayout(Radiant.Graphics2D.Renderer2D renderer, float scale)
    {
        var centerLineColor = new Vector4(1.0f, 1.0f, 1.0f, 0.3f);
        var debugLeftColor = new Vector4(0.0f, 1.0f, 1.0f, 0.5f);  // Cyan for n=-2.5
        var debugRightColor = new Vector4(1.0f, 0.5f, 0.0f, 0.5f); // Orange for n=+2.5

        // Draw dashed centerline along the entire track
        DrawDashedLine(renderer, 0.0, scale, centerLineColor);

        // Draw offset lines at n = -2.5 and n = +2.5 to show track width
        DrawOffsetLine(renderer, -2.5, scale, debugLeftColor);
        DrawOffsetLine(renderer, +2.5, scale, debugRightColor);
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
        // State is now [n, θ, v, T_f] - s is computed from frame position
        for (var i = 0; i < currentFrame && i < states.Length - 1; i++)
        {
            var s1 = totalFrames > 1 ? (i / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;
            var s2 = totalFrames > 1 ? ((i + 1) / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;
            var (x1, y1) = _trackGeometry.CurvilinearToCartesian(s1, states[i][0]);
            var (x2, y2) = _trackGeometry.CurvilinearToCartesian(s2, states[i + 1][0]);

            // Color by velocity (green = slow, yellow = medium, red = fast)
            var v = states[i][2];
            var vNorm = Math.Clamp((v - 5.0) / 20.0, 0.0, 1.0);
            var color = new Vector4((float)vNorm, (float)(1.0 - vNorm * 0.5), 0.2f, 0.9f);

            renderer.DrawLine(new Vector2((float)x1 * scale, (float)y1 * scale) + Translation,
                             new Vector2((float)x2 * scale, (float)y2 * scale) + Translation, color);
        }

        // Draw future path in gray
        for (var i = currentFrame; i < states.Length - 1; i++)
        {
            var s1 = totalFrames > 1 ? (i / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;
            var s2 = totalFrames > 1 ? ((i + 1) / (double)(totalFrames - 1)) * _trackGeometry.TotalLength : 0.0;
            var (x1, y1) = _trackGeometry.CurvilinearToCartesian(s1, states[i][0]);
            var (x2, y2) = _trackGeometry.CurvilinearToCartesian(s2, states[i + 1][0]);

            renderer.DrawLine(new Vector2((float)x1 * scale, (float)y1 * scale) + Translation,
                             new Vector2((float)x2 * scale, (float)y2 * scale) + Translation, new Vector4(0.4f, 0.4f, 0.4f, 0.5f));
        }
    }

    private static void DrawVehicle(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double heading, double velocity, float scale)
    {
        var vehX = (float)x * scale + Translation.X;
        var vehY = (float)y * scale + Translation.Y;

        // Draw vehicle as a triangle
        // Left-hand rule: positive θ = clockwise, so θ = +π/2 = south = negative y
        // Direction vector in Cartesian: (cos(θ), -sin(θ))
        const float VehicleSize = 12.0f;
        var frontX = vehX + (float)(VehicleSize * Math.Cos(heading));
        var frontY = vehY - (float)(VehicleSize * Math.Sin(heading));

        var angle1 = heading + 2.5 * Math.PI / 3.0;
        var angle2 = heading - 2.5 * Math.PI / 3.0;

        var backLeftX = vehX + (float)(VehicleSize * 0.7 * Math.Cos(angle1));
        var backLeftY = vehY - (float)(VehicleSize * 0.7 * Math.Sin(angle1));
        var backRightX = vehX + (float)(VehicleSize * 0.7 * Math.Cos(angle2));
        var backRightY = vehY - (float)(VehicleSize * 0.7 * Math.Sin(angle2));

        // Color by velocity
        var vNorm = Math.Clamp((velocity - 5.0) / 20.0, 0.0, 1.0);
        var vehicleColor = new Vector4((float)vNorm, (float)(1.0 - vNorm * 0.5), 0.3f, 1.0f);

        renderer.DrawLine(new Vector2(frontX, frontY), new Vector2(backLeftX, backLeftY), vehicleColor);
        renderer.DrawLine(new Vector2(backLeftX, backLeftY), new Vector2(backRightX, backRightY), vehicleColor);
        renderer.DrawLine(new Vector2(backRightX, backRightY), new Vector2(frontX, frontY), vehicleColor);

        renderer.DrawCircleFilled(vehX, vehY, 6, vehicleColor, 16);
    }

    private void DrawMarkers(Radiant.Graphics2D.Renderer2D renderer, double[][] states, float scale)
    {
        if (states.Length < 2) return;

        // State is now [n, θ, v, T_f] - s is computed from frame position
        // Start marker (s = 0)
        var (startX, startY) = _trackGeometry.CurvilinearToCartesian(0.0, states[0][0]);
        var startPos = new Vector2((float)startX * scale, (float)startY * scale) + Translation;
        renderer.DrawCircleFilled(startPos.X, startPos.Y, 10, Colors.Emerald500, 24);
        renderer.DrawText("START", startPos.X - 30, startPos.Y + 20, 2, Colors.Emerald400);

        // End marker (s = TotalLength)
        var (endX, endY) = _trackGeometry.CurvilinearToCartesian(_trackGeometry.TotalLength, states[^1][0]);
        var endPos = new Vector2((float)endX * scale, (float)endY * scale) + Translation;
        renderer.DrawCircleFilled(endPos.X, endPos.Y, 10, Colors.Rose500, 24);
        renderer.DrawText("FINISH", endPos.X - 35, endPos.Y - 15, 2, Colors.Rose400);
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer,
        double s, double n, double x, double y, double heading, double velocity,
        int iteration, double cost, double maxViolation, double constraintTolerance,
        int frameIndex, int totalFrames)
    {
        const float TopY = -370.0f;
        const float LeftX = -420.0f;
        const float RightX = 200.0f;

        // Left column
        renderer.DrawText($"ITERATION: {iteration}", LeftX, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"TIME: {cost:F3} s", LeftX, TopY + 20, 2, Colors.Sky400);

        var convergenceRatio = constraintTolerance > 0 ? maxViolation / constraintTolerance : 0.0;
        var isConverged = maxViolation < constraintTolerance;
        var convergenceColor = isConverged ? Colors.Emerald500 : (convergenceRatio < 10.0 ? Colors.Amber500 : Colors.Rose500);
        renderer.DrawText($"VIOLATION: {maxViolation:E2}", LeftX, TopY + 40, 2, convergenceColor);

        // Right column - curvilinear and Cartesian coordinates
        renderer.DrawText($"FRAME: {frameIndex + 1}/{totalFrames}", RightX, TopY, 2, Colors.Amber400);
        renderer.DrawText($"s={s:F1} n={n:F2}", RightX, TopY + 20, 2, Colors.Purple400);
        renderer.DrawText($"({x:F1}, {y:F1})", RightX, TopY + 40, 2, Colors.Purple300);
        renderer.DrawText($"V: {velocity:F1} m/s", RightX, TopY + 60, 2, Colors.Cyan400);

        var headingDeg = heading * 180.0 / Math.PI;
        renderer.DrawText($"HDG: {headingDeg:F0} DEG", RightX, TopY + 80, 2, Colors.Orange400);
    }
}
