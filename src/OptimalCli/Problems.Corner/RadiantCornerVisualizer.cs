/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Numerics;
using Radiant;

namespace OptimalCli.Problems.Corner;

/// <summary>
/// Renders a real-time graphical display of the corner problem using Radiant.
/// Shows the road layout, boundaries, and vehicle trajectory through the 90° turn.
/// Converts curvilinear coordinates (s, n) to Cartesian (x, y) for display.
/// </summary>
internal static class RadiantCornerVisualizer
{
    private const int WindowWidth = 900;
    private const int WindowHeight = 800;

    // Road geometry constants (from CornerDynamics)
    private static readonly double RoadHalfWidth = CornerDynamics.RoadHalfWidth;
    private static readonly double CenterlineRadius = CornerDynamics.CenterlineRadius;
    private static readonly double EntryLength = CornerDynamics.EntryLength;

    private static double[][]? s_currentStates;
    private static double[][]? s_currentControls;
    private static int s_currentIteration;
    private static double s_currentCost;
    private static double s_currentMaxViolation;
    private static double s_currentConstraintTolerance;
    private static int s_currentFrameIndex;
    private static DateTime s_animationStartTime;
    private static readonly object s_lock = new();

    // Buffered next trajectory
    private static double[][]? s_nextStates;
    private static double[][]? s_nextControls;
    private static int s_nextIteration;
    private static double s_nextCost;
    private static double s_nextMaxViolation;
    private static double s_nextConstraintTolerance;
    private static bool s_hasNextTrajectory;

    private static CancellationTokenSource? s_cancellationTokenSource;

    public static CancellationToken CancellationToken => s_cancellationTokenSource?.Token ?? CancellationToken.None;

    public static void UpdateTrajectory(double[][] states, double[][] controls, int iteration, double cost, double maxViolation, double constraintTolerance)
    {
        if (states.Length == 0 || controls.Length == 0)
        {
            return;
        }

        lock (s_lock)
        {
            s_nextStates = states;
            s_nextControls = controls;
            s_nextIteration = iteration;
            s_nextCost = cost;
            s_nextMaxViolation = maxViolation;
            s_nextConstraintTolerance = constraintTolerance;
            s_hasNextTrajectory = true;

            Console.WriteLine($"[VIZ] Buffered Iter {iteration}: {states.Length} frames, cost={cost:F4}, violation={maxViolation:E2}/{constraintTolerance:E2}");
        }
    }

    public static void RunVisualizationWindow()
    {
        s_cancellationTokenSource = new CancellationTokenSource();

        try
        {
            using var app = new RadiantApplication();
            app.Run("Corner Racing Line Optimization (Curvilinear)", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
        }
        finally
        {
            if (s_cancellationTokenSource?.IsCancellationRequested == false)
            {
                Console.WriteLine("[VIZ] Window closed - requesting optimization cancellation");
                s_cancellationTokenSource.Cancel();
            }

            s_cancellationTokenSource?.Dispose();
            s_cancellationTokenSource = null;
        }
    }

    /// <summary>
    /// Run visualization in debug mode - shows track and offset lines without optimization.
    /// </summary>
    public static void RunDebugVisualization()
    {
        Console.WriteLine("=== DEBUG VISUALIZATION MODE ===");
        Console.WriteLine("Showing track layout with offset lines at n = -2.5 (cyan) and n = +2.5 (orange)");
        Console.WriteLine("Close window to exit.");

        using var app = new RadiantApplication();
        app.Run("Corner Track Debug View", WindowWidth, WindowHeight, RenderDebugFrame, Colors.Slate900);
    }

    private static void RenderDebugFrame(Radiant.Graphics2D.Renderer2D renderer)
    {
        const float scale = 15.0f;

        // Draw road layout
        DrawRoadLayout(renderer, scale);

        // Draw debug info
        renderer.DrawText("DEBUG MODE - Track Layout", -400, -370, 2, Colors.Amber400);
        renderer.DrawText("Cyan line: n = -2.5 (left of centerline)", -400, -350, 2, Colors.Cyan400);
        renderer.DrawText("Orange line: n = +2.5 (right of centerline)", -400, -330, 2, Colors.Orange400);
        renderer.DrawText("Red lines: Road boundaries (n = ±5)", -400, -310, 2, Colors.Red400);
        renderer.DrawText("White dashed: Centerline (n = 0)", -400, -290, 2, Colors.White);
    }

    private static void RenderFrame(Radiant.Graphics2D.Renderer2D renderer)
    {
        double[][] states;
        double[][] controls;
        int iteration;
        double cost;
        double maxViolation;
        double constraintTolerance;
        int frameIndex;

        lock (s_lock)
        {
            if (s_currentStates == null || s_currentControls == null)
            {
                if (s_hasNextTrajectory && s_nextStates != null && s_nextControls != null)
                {
                    s_currentStates = s_nextStates;
                    s_currentControls = s_nextControls;
                    s_currentIteration = s_nextIteration;
                    s_currentCost = s_nextCost;
                    s_currentMaxViolation = s_nextMaxViolation;
                    s_currentConstraintTolerance = s_nextConstraintTolerance;
                    s_currentFrameIndex = 0;
                    s_animationStartTime = DateTime.Now;
                    s_hasNextTrajectory = false;

                    Console.WriteLine($"[VIZ] Started displaying Iter {s_currentIteration}");
                }
                else
                {
                    DrawRoadLayout(renderer, 15.0f);
                    renderer.DrawText("WAITING FOR OPTIMIZATION...", -200, 0, 3, Colors.Gray400);
                    return;
                }
            }

            var elapsed = (DateTime.Now - s_animationStartTime).TotalMilliseconds;
            const double FrameDuration = 80.0;
            var totalFrames = s_currentStates.Length;
            var frameInSequence = (int)(elapsed / FrameDuration);

            if (frameInSequence >= totalFrames && s_hasNextTrajectory && s_nextStates != null && s_nextControls != null)
            {
                s_currentStates = s_nextStates;
                s_currentControls = s_nextControls;
                s_currentIteration = s_nextIteration;
                s_currentCost = s_nextCost;
                s_currentMaxViolation = s_nextMaxViolation;
                s_currentConstraintTolerance = s_nextConstraintTolerance;
                s_currentFrameIndex = 0;
                s_animationStartTime = DateTime.Now;
                s_hasNextTrajectory = false;

                Console.WriteLine($"[VIZ] Switched to Iter {s_currentIteration}");
                frameInSequence = 0;
                totalFrames = s_currentStates.Length;
            }

            s_currentFrameIndex = frameInSequence % totalFrames;

            states = s_currentStates;
            controls = s_currentControls;
            iteration = s_currentIteration;
            cost = s_currentCost;
            maxViolation = s_currentMaxViolation;
            constraintTolerance = s_currentConstraintTolerance;
            frameIndex = s_currentFrameIndex;
        }

        // Calculate scale to fit the road in the window
        const float Scale = 15.0f;

        // Draw road layout first (background)
        DrawRoadLayout(renderer, Scale);

        // Draw trajectory path (convert curvilinear to Cartesian)
        DrawPathTrace(renderer, states, frameIndex, Scale);

        // Draw the vehicle (convert curvilinear to Cartesian)
        var state = states[frameIndex];
        var s = state[0];
        var n = state[1];
        var theta = state[2];
        var v = state[3];
        var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(s, n);
        DrawVehicle(renderer, x, y, theta, v, Scale);

        // Draw start and end markers
        DrawMarkers(renderer, states, Scale);

        // Draw information overlay
        var control = controls[frameIndex];
        DrawInformation(renderer, s, n, x, y, theta, v, control[0], control[1],
            iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length);
    }

    private static void DrawRoadLayout(Radiant.Graphics2D.Renderer2D renderer, float scale)
    {
        var boundaryColor = Colors.Red500;
        var centerLineColor = new Vector4(1.0f, 1.0f, 1.0f, 0.3f);

        // Road geometry for curvilinear coordinates:
        // - Entry: s ∈ [0, EntryLength), centerline at y=0, x = s - EntryLength
        //   Inner edge at y = +RoadHalfWidth (n = -RoadHalfWidth in curvilinear)
        //   Outer edge at y = -RoadHalfWidth (n = +RoadHalfWidth in curvilinear)
        //   Wait, let me check the coordinate system...
        //
        // From CurvilinearToCartesian for entry: x = s - EntryLength, y = -n
        // So n > 0 means y < 0 (right side of road when facing east)
        // Road bounds: |n| <= RoadHalfWidth, so |y| <= RoadHalfWidth
        //
        // Entry straight: from x = -EntryLength to x = 0, y ∈ [-RoadHalfWidth, +RoadHalfWidth]

        // === ENTRY STRAIGHT ===
        // Lower edge: y = -RoadHalfWidth
        renderer.DrawLine(new Vector2(-(float)EntryLength * scale, (float)RoadHalfWidth * scale),
                         new Vector2(0, (float)RoadHalfWidth * scale), boundaryColor);
        // Upper edge: y = +RoadHalfWidth
        renderer.DrawLine(new Vector2(-(float)EntryLength * scale, -(float)RoadHalfWidth * scale),
                         new Vector2(0, -(float)RoadHalfWidth * scale), boundaryColor);

        // === ARC SECTION ===
        // Arc centerline is at (0,0) start, curves to (CenterlineRadius, -CenterlineRadius)
        // Inner edge: radius = CenterlineRadius - RoadHalfWidth = 0 (point!)
        // Outer edge: radius = CenterlineRadius + RoadHalfWidth = 10
        const int ArcSegments = 30;
        
        // Inner arc (at n = -RoadHalfWidth)
        // This is actually a point at the apex when CenterlineRadius == RoadHalfWidth
        if (Math.Abs(CenterlineRadius - RoadHalfWidth) > 0.1)
        {
            for (var i = 0; i < ArcSegments; i++)
            {
                var (ix1, iy1) = GetArcBoundaryPoint(i, ArcSegments, -RoadHalfWidth);
                var (ix2, iy2) = GetArcBoundaryPoint(i + 1, ArcSegments, -RoadHalfWidth);
                renderer.DrawLine(new Vector2((float)ix1 * scale, -(float)iy1 * scale),
                                 new Vector2((float)ix2 * scale, -(float)iy2 * scale), boundaryColor);
            }
        }
        else
        {
            // Inner boundary is a single point - just mark it
            var (apexX, apexY) = GetArcBoundaryPoint(ArcSegments / 2, ArcSegments, -RoadHalfWidth + 0.01);
            renderer.DrawCircleOutline((float)apexX * scale, -(float)apexY * scale, 8, Colors.Amber400, 16);
            renderer.DrawText("APEX", (float)apexX * scale + 12, -(float)apexY * scale + 5, 2, Colors.Amber400);
        }

        // Outer arc (at n = +RoadHalfWidth)
        for (var i = 0; i < ArcSegments; i++)
        {
            var (ox1, oy1) = GetArcBoundaryPoint(i, ArcSegments, RoadHalfWidth);
            var (ox2, oy2) = GetArcBoundaryPoint(i + 1, ArcSegments, RoadHalfWidth);
            renderer.DrawLine(new Vector2((float)ox1 * scale, -(float)oy1 * scale),
                             new Vector2((float)ox2 * scale, -(float)oy2 * scale), boundaryColor);
        }

        // === EXIT STRAIGHT ===
        // From CurvilinearToCartesian for exit: x = CenterlineRadius + n, y = -CenterlineRadius - exitProgress
        // Left edge: n = -RoadHalfWidth → x = CenterlineRadius - RoadHalfWidth = 0
        // Right edge: n = +RoadHalfWidth → x = CenterlineRadius + RoadHalfWidth = 10
        var exitLeftX = CenterlineRadius - RoadHalfWidth;
        var exitRightX = CenterlineRadius + RoadHalfWidth;
        var exitStartY = -CenterlineRadius;
        var exitEndY = -CenterlineRadius - 25.0; // Show 25m of exit

        renderer.DrawLine(new Vector2((float)exitLeftX * scale, -(float)exitStartY * scale),
                         new Vector2((float)exitLeftX * scale, -(float)exitEndY * scale), boundaryColor);
        renderer.DrawLine(new Vector2((float)exitRightX * scale, -(float)exitStartY * scale),
                         new Vector2((float)exitRightX * scale, -(float)exitEndY * scale), boundaryColor);

        // === CENTER LINE (dashed) ===
        // Entry center
        for (var xc = -(float)EntryLength * scale; xc < 0; xc += 20)
        {
            renderer.DrawLine(new Vector2(xc, 0), new Vector2(Math.Min(xc + 10, 0), 0), centerLineColor);
        }
        // Arc center
        for (var i = 0; i < ArcSegments; i++)
        {
            if (i % 2 == 0)
            {
                var (cx1, cy1) = GetArcBoundaryPoint(i, ArcSegments, 0);
                var (cx2, cy2) = GetArcBoundaryPoint(i + 1, ArcSegments, 0);
                renderer.DrawLine(new Vector2((float)cx1 * scale, -(float)cy1 * scale),
                                 new Vector2((float)cx2 * scale, -(float)cy2 * scale), centerLineColor);
            }
        }
        // Exit center
        var exitCenterX = CenterlineRadius;
        for (var yc = (float)exitStartY; yc > (float)exitEndY; yc -= 20.0f / scale)
        {
            renderer.DrawLine(new Vector2((float)exitCenterX * scale, -yc * scale),
                             new Vector2((float)exitCenterX * scale, -Math.Max(yc - 10.0f / scale, (float)exitEndY) * scale), centerLineColor);
        }

        // === DEBUG: Draw offset lines at n = -2.5 and n = +2.5 to verify curvilinear coords ===
        var debugLeftColor = new Vector4(0.0f, 1.0f, 1.0f, 0.5f);  // Cyan for n=-2.5
        var debugRightColor = new Vector4(1.0f, 0.5f, 0.0f, 0.5f); // Orange for n=+2.5
        DrawOffsetLine(renderer, -2.5, scale, debugLeftColor);
        DrawOffsetLine(renderer, +2.5, scale, debugRightColor);
    }

    /// <summary>
    /// Draw a line at a fixed n offset along the entire track using CurvilinearToCartesian.
    /// This verifies the coordinate conversion is correct.
    /// </summary>
    private static void DrawOffsetLine(Radiant.Graphics2D.Renderer2D renderer, double nOffset, float scale, Vector4 color)
    {
        const int NumPoints = 100;
        var arcLength = CornerDynamics.ArcLength;
        var totalLength = EntryLength + arcLength + 25.0; // Entry + arc + 25m exit

        for (var i = 0; i < NumPoints - 1; i++)
        {
            var s1 = totalLength * i / (NumPoints - 1);
            var s2 = totalLength * (i + 1) / (NumPoints - 1);

            var (x1, y1) = CornerDynamicsHelpers.CurvilinearToCartesian(s1, nOffset);
            var (x2, y2) = CornerDynamicsHelpers.CurvilinearToCartesian(s2, nOffset);

            renderer.DrawLine(
                new Vector2((float)x1 * scale, -(float)y1 * scale),
                new Vector2((float)x2 * scale, -(float)y2 * scale),
                color);
        }
    }

    private static (double x, double y) GetArcBoundaryPoint(int segment, int totalSegments, double nOffset)
    {
        var arcLength = CornerDynamics.ArcLength;
        var entryEnd = EntryLength;
        var arcProgress = (double)segment / totalSegments;
        var s = entryEnd + arcProgress * arcLength;
        return CornerDynamicsHelpers.CurvilinearToCartesian(s, nOffset);
    }

    private static void DrawPathTrace(Radiant.Graphics2D.Renderer2D renderer, double[][] states, int currentFrame, float scale)
    {
        // Draw completed path
        for (var i = 0; i < currentFrame && i < states.Length - 1; i++)
        {
            var (x1, y1) = CornerDynamicsHelpers.CurvilinearToCartesian(states[i][0], states[i][1]);
            var (x2, y2) = CornerDynamicsHelpers.CurvilinearToCartesian(states[i + 1][0], states[i + 1][1]);

            // Color by velocity (green = slow, yellow = medium, red = fast)
            var v = states[i][3];
            var vNorm = Math.Clamp((v - 5.0) / 20.0, 0.0, 1.0);
            var color = new Vector4((float)vNorm, (float)(1.0 - vNorm * 0.5), 0.2f, 0.9f);

            renderer.DrawLine(new Vector2((float)x1 * scale, -(float)y1 * scale),
                             new Vector2((float)x2 * scale, -(float)y2 * scale), color);
        }

        // Draw future path in gray
        for (var i = currentFrame; i < states.Length - 1; i++)
        {
            var (x1, y1) = CornerDynamicsHelpers.CurvilinearToCartesian(states[i][0], states[i][1]);
            var (x2, y2) = CornerDynamicsHelpers.CurvilinearToCartesian(states[i + 1][0], states[i + 1][1]);

            renderer.DrawLine(new Vector2((float)x1 * scale, -(float)y1 * scale),
                             new Vector2((float)x2 * scale, -(float)y2 * scale), new Vector4(0.4f, 0.4f, 0.4f, 0.5f));
        }
    }

    private static void DrawVehicle(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double heading, double velocity, float scale)
    {
        var vehX = (float)x * scale;
        var vehY = -(float)y * scale;

        // Draw vehicle as a triangle
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

    private static void DrawMarkers(Radiant.Graphics2D.Renderer2D renderer, double[][] states, float scale)
    {
        if (states.Length < 2) return;

        // Start marker
        var (startX, startY) = CornerDynamicsHelpers.CurvilinearToCartesian(states[0][0], states[0][1]);
        renderer.DrawCircleFilled((float)startX * scale, -(float)startY * scale, 10, Colors.Emerald500, 24);
        renderer.DrawText("START", (float)startX * scale - 30, -(float)startY * scale - 20, 2, Colors.Emerald400);

        // End marker
        var (endX, endY) = CornerDynamicsHelpers.CurvilinearToCartesian(states[^1][0], states[^1][1]);
        renderer.DrawCircleFilled((float)endX * scale, -(float)endY * scale, 10, Colors.Rose500, 24);
        renderer.DrawText("FINISH", (float)endX * scale - 35, -(float)endY * scale + 15, 2, Colors.Rose400);
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer,
        double s, double n, double x, double y, double heading, double velocity,
        double accel, double steerRate,
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
