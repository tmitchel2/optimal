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
/// </summary>
internal static class RadiantCornerVisualizer
{
    private const int WindowWidth = 900;
    private const int WindowHeight = 800;

    // Road geometry constants (must match CornerProblemSolver)
    private const double RoadWidth = 10.0;
    private const double HalfRoadWidth = RoadWidth / 2.0;
    // Corner center is at the inner corner where entry and exit meet
    private const double CornerCenterX = HalfRoadWidth;  // = 5
    private const double CornerCenterY = -HalfRoadWidth; // = -5
    private const double CornerRadius = RoadWidth;       // = 10 (outer boundary arc radius)

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
            app.Run("Corner Racing Line Optimization", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
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

        // Draw trajectory path
        DrawPathTrace(renderer, states, frameIndex, Scale);

        // Draw the vehicle
        var state = states[frameIndex];
        DrawVehicle(renderer, state[0], state[1], state[2], state[3], Scale);

        // Draw start and end markers
        DrawMarkers(renderer, states, Scale);

        // Draw information overlay
        var control = controls[frameIndex];
        DrawInformation(renderer, state[0], state[1], state[2], state[3], control[0], control[1],
            iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length);
    }

    private static void DrawRoadLayout(Radiant.Graphics2D.Renderer2D renderer, float scale)
    {
        var boundaryColor = Colors.Red500;
        var centerLineColor = new Vector4(1.0f, 1.0f, 1.0f, 0.3f);

        // Screen coordinate: screen_y = -world_y * scale (matching trajectory drawing)
        // +Y screen is UP in Radiant
        //
        // NEW Road geometry (matching solver constraints):
        //   Entry: x <= 0, y ∈ [-HalfRoadWidth, +HalfRoadWidth] = [-5, +5]
        //   Corner: x > 0 AND y > -HalfRoadWidth, outer arc centered at (5, -5) with radius 10
        //   Exit: y <= -HalfRoadWidth = -5, x ∈ [HalfRoadWidth, HalfRoadWidth + RoadWidth] = [5, 15]
        //
        // Inner corner is a sharp 90° angle at (5, -5) world = (5, +5) screen
        // Outer corner is a quarter-circle arc from (0, +5) world to (15, -5) world

        // === ENTRY STRAIGHT ===
        // Lower edge: y = -HalfRoadWidth = -5 world → +5 screen, from x=-25 to x=0
        renderer.DrawLine(new Vector2(-25.0f * scale, (float)HalfRoadWidth * scale),
                         new Vector2(0, (float)HalfRoadWidth * scale), boundaryColor);
        // Upper edge: y = +HalfRoadWidth = +5 world → -5 screen, from x=-25 to x=0
        renderer.DrawLine(new Vector2(-25.0f * scale, -(float)HalfRoadWidth * scale),
                         new Vector2(0, -(float)HalfRoadWidth * scale), boundaryColor);

        // === EXIT STRAIGHT ===
        // Exit is at y <= -HalfRoadWidth = -5 world → +5 screen and below (increasing screen y)
        // Left edge: x = HalfRoadWidth = 5, from y=-5 world to y=-40 world (screen y=+5 to +40)
        renderer.DrawLine(new Vector2((float)HalfRoadWidth * scale, (float)HalfRoadWidth * scale),
                         new Vector2((float)HalfRoadWidth * scale, 40.0f * scale), boundaryColor);
        // Right edge: x = HalfRoadWidth + RoadWidth = 15, from y=-5 world to y=-40 world
        renderer.DrawLine(new Vector2((float)(HalfRoadWidth + RoadWidth) * scale, (float)HalfRoadWidth * scale),
                         new Vector2((float)(HalfRoadWidth + RoadWidth) * scale, 40.0f * scale), boundaryColor);

        // === INNER CORNER (sharp 90° angle) ===
        // The inner boundary goes: entry lower at (0, -5) → corner at (5, -5) → exit left at (5, -5)
        // In screen coords: (0, +5) → (5, +5) horizontal, then (5, +5) → (5, +40) vertical
        // The corner point is at (5, +5) screen = (5, -5) world
        renderer.DrawLine(new Vector2(0, (float)HalfRoadWidth * scale),
                         new Vector2((float)HalfRoadWidth * scale, (float)HalfRoadWidth * scale), boundaryColor);

        // === OUTER CORNER (quarter-circle arc) ===
        // Arc centered at (CornerCenterX, CornerCenterY) = (5, -5) world = (5, +5) screen
        // Radius = CornerRadius = 10
        // Arc goes from angle 90° (pointing up in world, connects to entry upper at (0, +5) world)
        // to angle 0° (pointing right in world, connects to exit right at (15, -5) world)
        const int ArcSegments = 30;
        for (var i = 0; i < ArcSegments; i++)
        {
            // Angles from 90° down to 0° (counterclockwise in screen coords because of Y flip)
            var angle1 = Math.PI / 2.0 * (1.0 - (double)i / ArcSegments);
            var angle2 = Math.PI / 2.0 * (1.0 - (double)(i + 1) / ArcSegments);
            
            // World coords: center + radius * (cos, sin)
            // Screen y = -world_y
            var world_x1 = CornerCenterX + CornerRadius * Math.Cos(angle1);
            var world_y1 = CornerCenterY + CornerRadius * Math.Sin(angle1);
            var world_x2 = CornerCenterX + CornerRadius * Math.Cos(angle2);
            var world_y2 = CornerCenterY + CornerRadius * Math.Sin(angle2);
            
            var screen_x1 = (float)world_x1 * scale;
            var screen_y1 = -(float)world_y1 * scale;
            var screen_x2 = (float)world_x2 * scale;
            var screen_y2 = -(float)world_y2 * scale;
            
            renderer.DrawLine(new Vector2(screen_x1, screen_y1), new Vector2(screen_x2, screen_y2), boundaryColor);
        }

        // === CENTER LINE (dashed) ===
        // Entry center (y=0 world = 0 screen)
        for (var x = -25.0f * scale; x < 0; x += 20)
        {
            renderer.DrawLine(new Vector2(x, 0), new Vector2(Math.Min(x + 10, 0), 0), centerLineColor);
        }
        // Corner center arc (radius = 5, centered at (5, -5) world = (5, +5) screen)
        // This gives the center of the road through the corner
        var centerRadius = RoadWidth / 2.0; // = 5
        for (var i = 0; i < ArcSegments; i++)
        {
            if (i % 2 == 0)
            {
                var angle1 = Math.PI / 2.0 * (1.0 - (double)i / ArcSegments);
                var angle2 = Math.PI / 2.0 * (1.0 - (double)(i + 1) / ArcSegments);
                
                var world_x1 = CornerCenterX + centerRadius * Math.Cos(angle1);
                var world_y1 = CornerCenterY + centerRadius * Math.Sin(angle1);
                var world_x2 = CornerCenterX + centerRadius * Math.Cos(angle2);
                var world_y2 = CornerCenterY + centerRadius * Math.Sin(angle2);
                
                renderer.DrawLine(
                    new Vector2((float)world_x1 * scale, -(float)world_y1 * scale),
                    new Vector2((float)world_x2 * scale, -(float)world_y2 * scale),
                    centerLineColor);
            }
        }
        // Exit center (x = HalfRoadWidth + RoadWidth/2 = 10, from y=-5 to y=-40 world)
        var exitCenterX = HalfRoadWidth + RoadWidth / 2.0; // = 10
        for (var y = (float)HalfRoadWidth * scale; y < 40.0f * scale; y += 20)
        {
            renderer.DrawLine(new Vector2((float)exitCenterX * scale, y), 
                             new Vector2((float)exitCenterX * scale, Math.Min(y + 10, 40.0f * scale)), centerLineColor);
        }

        // === APEX marker ===
        // The apex (clipping point) is at the inner corner at (5, -5) world = (5, +5) screen
        var apexScreenX = (float)CornerCenterX * scale;
        var apexScreenY = -(float)CornerCenterY * scale;  // -(-5) = +5
        renderer.DrawCircleOutline(apexScreenX, apexScreenY, 8, Colors.Amber400, 16);
        renderer.DrawText("APEX", apexScreenX + 12, apexScreenY + 5, 2, Colors.Amber400);
    }

    private static void DrawPathTrace(Radiant.Graphics2D.Renderer2D renderer, double[][] states, int currentFrame, float scale)
    {
        // Draw completed path
        for (var i = 0; i < currentFrame && i < states.Length - 1; i++)
        {
            var x1 = (float)states[i][0] * scale;
            var y1 = -(float)states[i][1] * scale;
            var x2 = (float)states[i + 1][0] * scale;
            var y2 = -(float)states[i + 1][1] * scale;

            // Color by velocity (green = slow, yellow = medium, red = fast)
            var v = states[i][3];
            var vNorm = Math.Clamp((v - 5.0) / 20.0, 0.0, 1.0);
            var color = new Vector4((float)vNorm, (float)(1.0 - vNorm * 0.5), 0.2f, 0.9f);

            renderer.DrawLine(new Vector2(x1, y1), new Vector2(x2, y2), color);
        }

        // Draw future path in gray
        for (var i = currentFrame; i < states.Length - 1; i++)
        {
            var x1 = (float)states[i][0] * scale;
            var y1 = -(float)states[i][1] * scale;
            var x2 = (float)states[i + 1][0] * scale;
            var y2 = -(float)states[i + 1][1] * scale;

            renderer.DrawLine(new Vector2(x1, y1), new Vector2(x2, y2), new Vector4(0.4f, 0.4f, 0.4f, 0.5f));
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
        var startX = (float)states[0][0] * scale;
        var startY = -(float)states[0][1] * scale;
        renderer.DrawCircleFilled(startX, startY, 10, Colors.Emerald500, 24);
        renderer.DrawText("START", startX - 30, startY - 20, 2, Colors.Emerald400);

        // End marker
        var endX = (float)states[^1][0] * scale;
        var endY = -(float)states[^1][1] * scale;
        renderer.DrawCircleFilled(endX, endY, 10, Colors.Rose500, 24);
        renderer.DrawText("FINISH", endX - 35, endY + 15, 2, Colors.Rose400);
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer,
        double x, double y, double heading, double velocity,
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

        // Right column
        renderer.DrawText($"FRAME: {frameIndex + 1}/{totalFrames}", RightX, TopY, 2, Colors.Amber400);
        renderer.DrawText($"POS: ({x:F1}, {y:F1})", RightX, TopY + 20, 2, Colors.Purple400);
        renderer.DrawText($"V: {velocity:F1} m/s", RightX, TopY + 40, 2, Colors.Cyan400);

        var headingDeg = heading * 180.0 / Math.PI;
        renderer.DrawText($"HDG: {headingDeg:F0} DEG", RightX, TopY + 60, 2, Colors.Orange400);
    }
}
