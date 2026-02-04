/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Numerics;
using Radiant;

namespace OptimalCli.Problems.Goddard;

/// <summary>
/// Renders a real-time graphical display of the Goddard rocket using Radiant.
/// </summary>
internal static class RadiantGoddardRocketVisualizer
{
    private const int WindowWidth = 1000;
    private const int WindowHeight = 700;
    private const float RocketWidth = 7.5f;   // 1/4 of original 30
    private const float RocketHeight = 15.0f; // 1/4 of original 60
    private const float GroundY = -WindowHeight / 2.0f + 30.0f;  // Near bottom of screen

    private static double[][]? s_currentStates;
    private static double[][]? s_currentControls;
    private static int s_currentIteration;
    private static double s_currentCost;
    private static double s_currentMaxViolation;
    private static double s_currentConstraintTolerance;
    private static double s_currentScaleHeight;
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
    private static double s_nextScaleHeight;
    private static bool s_hasNextTrajectory;

    public static void UpdateTrajectory(double[][] states, double[][] controls, double[] times, double[][]? derivatives, int iteration, double cost, double maxViolation, double constraintTolerance, double scaleHeight)
    {
        // times and derivatives are available for smooth interpolation but not used in this visualizer
        _ = times;
        _ = derivatives;

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
            s_nextScaleHeight = scaleHeight;
            s_hasNextTrajectory = true;

            Console.WriteLine($"[VIZ] Buffered Iter {iteration}: {states.Length} frames, cost={cost:F4}, violation={maxViolation:E2}/{constraintTolerance:E2}");
        }
    }

    public static void RunVisualizationWindow(CancellationTokenSource cancellationTokenSource)
    {
        try
        {
            using var app = new RadiantApplication();
            app.Run("Goddard Rocket Optimization", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
        }
        finally
        {
            if (!cancellationTokenSource.IsCancellationRequested)
            {
                Console.WriteLine("[VIZ] Window closed - requesting optimization cancellation");
                cancellationTokenSource.Cancel();
            }
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
                    s_currentScaleHeight = s_nextScaleHeight;
                    s_currentFrameIndex = 0;
                    s_animationStartTime = DateTime.Now;
                    s_hasNextTrajectory = false;

                    Console.WriteLine($"[VIZ] Started displaying Iter {s_currentIteration}");
                }
                else
                {
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
                s_currentScaleHeight = s_nextScaleHeight;
                s_currentFrameIndex = 0;
                s_animationStartTime = DateTime.Now;
                s_hasNextTrajectory = false;

                Console.WriteLine($"[VIZ] Switched to Iter {s_currentIteration} after completing previous animation");

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

        var state = states[frameIndex];
        var control = controls[frameIndex];

        var altitude = state[0];
        var velocity = state[1];
        var mass = state[2];
        var thrust = control[0];

        // Draw ground
        DrawGround(renderer);

        // Calculate current trajectory's max altitude for scaling
        var currentMaxAltitude = states.Max(s => s[0]);

        // Draw trajectory path
        DrawTrajectory(renderer, states, currentMaxAltitude);

        // Draw rocket
        DrawRocket(renderer, altitude, thrust, mass, currentMaxAltitude, states);

        // Draw state and control graphs
        DrawGraphs(renderer, states, controls, frameIndex);

        // Draw information
        DrawInformation(renderer, altitude, velocity, mass, thrust, iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length, states);
    }

    private static void DrawGround(Radiant.Graphics2D.Renderer2D renderer)
    {
        const float GroundThickness = 6.0f;
        renderer.DrawRectangleFilled(-WindowWidth / 2, GroundY - GroundThickness / 2, WindowWidth, GroundThickness, Colors.Emerald800);

        // Draw ground texture
        for (var x = -WindowWidth / 2; x < WindowWidth / 2; x += 20)
        {
            renderer.DrawLine(
                new Vector2(x, GroundY - GroundThickness / 2),
                new Vector2(x + 10, GroundY - GroundThickness / 2 - 4),
                Colors.Emerald700);
        }
    }

    private static void DrawTrajectory(Radiant.Graphics2D.Renderer2D renderer, double[][] states, double currentMaxAltitude)
    {
        // Calculate dynamic scale based on current trajectory's max altitude
        var maxDisplayHeight = WindowHeight / 2.0 - GroundY - 100.0;  // Leave room for rocket at top
        var scaleMax = Math.Max(currentMaxAltitude, 1.0);  // Avoid division by zero
        var dynamicScale = maxDisplayHeight / scaleMax;

        // Draw the altitude trajectory as a vertical path
        // +Y is up, so add altitude to go higher
        for (var i = 1; i < states.Length; i++)
        {
            var h1 = (float)(states[i - 1][0] * dynamicScale);
            var h2 = (float)(states[i][0] * dynamicScale);

            var y1 = GroundY + h1;  // Higher altitude = more positive Y
            var y2 = GroundY + h2;

            var alpha = 0.3f + 0.4f * (float)i / states.Length;
            var color = new Vector4(0.4f, 0.6f, 0.9f, alpha);

            renderer.DrawLine(
                new Vector2(-150, y1),
                new Vector2(-150, y2),
                color);
        }
    }

    private static void DrawRocket(Radiant.Graphics2D.Renderer2D renderer, double altitude, double thrust, double mass, double currentMaxAltitude, double[][] states)
    {
        // Calculate dynamic scale based on current trajectory's max altitude
        var maxDisplayHeight = WindowHeight / 2.0 - GroundY - 100.0;  // Leave room for rocket at top
        var scaleMax = Math.Max(currentMaxAltitude, 1.0);  // Avoid division by zero
        var dynamicScale = maxDisplayHeight / scaleMax;

        // Radiant left-hand: +Y is UP
        // Higher altitude = move up on screen = more positive Y
        // Offset by RocketHeight/2 so bottom of rocket touches ground at altitude=0
        var rocketY = GroundY + RocketHeight / 2 + (float)(altitude * dynamicScale);
        const float RocketX = 0.0f;

        // Rocket body centered at rocketY
        var bodyBottom = rocketY - RocketHeight / 2;  // Bottom edge (less Y = lower)
        var bodyTop = rocketY + RocketHeight / 2;     // Top edge (more Y = higher)

        // Draw rocket body
        renderer.DrawRectangleFilled(
            RocketX - RocketWidth / 2,
            bodyBottom,
            RocketWidth,
            RocketHeight,
            Colors.Gray200);

        // Draw rocket outline
        renderer.DrawRectangleOutline(
            RocketX - RocketWidth / 2,
            bodyBottom,
            RocketWidth,
            RocketHeight,
            Colors.Gray400);

        // Nose cone: should point UP (positive Y direction) from top of body
        var noseHeight = 5.0f;  // 1/4 of original 20

        renderer.DrawRectangleFilled(
            RocketX - RocketWidth / 4,
            bodyTop,  // Start at top of body
            RocketWidth / 2,
            noseHeight,  // Extends upward
            Colors.Red600);

        // Nose tip
        renderer.DrawRectangleFilled(
            RocketX - 0.5f,
            bodyTop + noseHeight,  // Above the nose cone
            1,
            1.25f,  // 1/4 of original 5
            Colors.Red700);

        // Fins: should extend DOWN (negative Y direction) from bottom of body
        var finHeight = 3.75f;  // 1/4 of original 15
        var finWidth = 3.0f;    // 1/4 of original 12

        // Left fin
        renderer.DrawRectangleFilled(
            RocketX - RocketWidth / 2 - finWidth,
            bodyBottom - finHeight,  // Below body bottom
            finWidth,
            finHeight,
            Colors.Red700);

        // Right fin
        renderer.DrawRectangleFilled(
            RocketX + RocketWidth / 2,
            bodyBottom - finHeight,  // Below body bottom
            finWidth,
            finHeight,
            Colors.Red700);

        // Draw thrust flame (using multiple ellipses for flame effect)
        // Flame goes downward (negative Y direction) from bottom of rocket
        if (thrust > 0.1)
        {
            var flameIntensity = (float)(thrust / 3.1);  // Normalized by max thrust
            var flameHeight = 7.5f * flameIntensity;  // 1/4 of original 30
            var flameWidth = RocketWidth * 0.7f;

            // Draw outer flame (orange) - ellipses extending downward
            for (var i = 0; i < 3; i++)
            {
                var offsetY = bodyBottom - (flameHeight / 3.0f) * (i + 1);  // Decreasing Y = down
                var widthScale = 1.0f - (i * 0.3f);
                renderer.DrawEllipseFilled(
                    RocketX,
                    offsetY,
                    flameWidth / 2 * widthScale,
                    flameHeight / 6,
                    new Vector4(1.0f, 0.5f, 0.0f, 0.8f - i * 0.2f),
                    16);
            }

            // Inner flame (yellow)
            for (var i = 0; i < 2; i++)
            {
                var offsetY = bodyBottom - (flameHeight / 4.0f) * (i + 1);  // Decreasing Y = down
                var widthScale = 1.0f - (i * 0.4f);
                renderer.DrawEllipseFilled(
                    RocketX,
                    offsetY,
                    flameWidth / 4 * widthScale,
                    flameHeight / 8,
                    new Vector4(1.0f, 1.0f, 0.3f, 0.9f - i * 0.3f),
                    12);
            }
        }

        // Draw fuel indicator next to rocket (vertical bar)
        // Calculate mass percentage dynamically based on trajectory's mass range
        var masses = states.Select(s => s[2]).ToArray();
        var maxMass = masses.Max();  // Initial mass (full fuel)
        var minMass = masses.Min();  // Final mass (empty or near-empty)
        var massRange = maxMass - minMass;
        var massPercent = massRange > 1e-6 ? (float)((mass - minMass) / massRange * 100) : 100.0f;
        massPercent = Math.Clamp(massPercent, 0.0f, 100.0f);
        
        var massBarHeight = RocketHeight * 0.6f;
        var massBarWidth = 1.0f;  // 1/4 of original 4
        var massBarX = RocketX + RocketWidth / 2 + 1.25f;  // 1/4 of original 5
        var massBarY = rocketY;

        // Background (full bar) - centered at rocket Y
        var barBottom = massBarY - massBarHeight / 2;
        renderer.DrawRectangleFilled(massBarX, barBottom, massBarWidth, massBarHeight, Colors.Slate700);

        // Fuel level fills from bottom up
        var fuelHeight = massBarHeight * (massPercent / 100.0f);
        renderer.DrawRectangleFilled(
            massBarX,
            barBottom,  // Start at bottom of bar
            massBarWidth,
            fuelHeight,  // Fill upward
            massPercent > 30 ? Colors.Emerald500 : (massPercent > 10 ? Colors.Amber500 : Colors.Red500));
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer, double altitude, double velocity, double mass, double thrust, int iteration, double cost, double maxViolation, double constraintTolerance, int frameIndex, int totalFrames, double[][] states)
    {
        const float TopY = 280.0f;   // Top of screen (positive Y is up)
        const float RightX = 250.0f;

        // Left column - optimization info
        renderer.DrawText($"ITERATION: {iteration}", -480, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"COST: {cost:F4}", -480, TopY - 20, 2, Colors.Sky400);

        var convergenceRatio = constraintTolerance > 0 ? maxViolation / constraintTolerance : 0.0;
        var isConverged = maxViolation < constraintTolerance;
        var convergenceColor = isConverged ? Colors.Emerald500 : (convergenceRatio < 10.0 ? Colors.Amber500 : Colors.Rose500);
        renderer.DrawText($"CONVERGENCE: {maxViolation:E2}/{constraintTolerance:E2}", -480, TopY - 40, 2, convergenceColor);

        // Convergence progress bar
        const float ConvBarWidth = 200.0f;
        const float ConvBarHeight = 8.0f;
        const float ConvBarX = -480.0f;
        const float ConvBarY = TopY - 60.0f;

        renderer.DrawRectangleFilled(ConvBarX, ConvBarY, ConvBarWidth, ConvBarHeight, Colors.Slate700);
        var progress = Math.Min(1.0, Math.Max(0.0, 1.0 - Math.Log10(Math.Max(0.1, convergenceRatio)) / 2.0));
        var progressWidth = (float)(progress * ConvBarWidth);
        renderer.DrawRectangleFilled(ConvBarX, ConvBarY, progressWidth, ConvBarHeight, convergenceColor);

        // Right column - rocket state
        renderer.DrawText($"FRAME: {frameIndex + 1}/{totalFrames}", RightX, TopY, 2, Colors.Amber400);
        renderer.DrawText($"ALTITUDE: {altitude:F1} M", RightX, TopY - 20, 2, Colors.Purple400);
        renderer.DrawText($"VELOCITY: {velocity:F2} M/S", RightX, TopY - 40, 2, Colors.Cyan400);
        renderer.DrawText($"MASS: {mass:F3} KG", RightX, TopY - 60, 2, Colors.Emerald400);
        renderer.DrawText($"THRUST: {thrust:F2} N", RightX, TopY - 80, 2, Colors.Rose400);

        var masses = states.Select(s => s[2]).ToArray();
        var maxMass = masses.Max();
        var minMass = masses.Min();
        var massRange = maxMass - minMass;
        var fuelPercent = massRange > 1e-6 ? (mass - minMass) / massRange * 100 : 100.0;
        fuelPercent = Math.Clamp(fuelPercent, 0.0, 100.0);
        var fuelColor = fuelPercent > 30 ? Colors.Emerald500 : (fuelPercent > 10 ? Colors.Amber500 : Colors.Red500);
        var fuelText = fuelPercent > 0.1 ? $"FUEL: {fuelPercent:F1}%" : "FUEL: EMPTY";
        renderer.DrawText(fuelText, RightX, TopY - 100, 2, fuelColor);

        // Status
        var statusText = velocity > 0 ? "ASCENDING" : (velocity < -0.5 ? "DESCENDING" : "COASTING");
        var statusColor = velocity > 0 ? Colors.Emerald500 : (velocity < -0.5 ? Colors.Rose500 : Colors.Amber500);

        // Override status if fuel is depleted and coasting
        if (fuelPercent < 0.1 && thrust < 0.01)
        {
            statusText = velocity > 0 ? "COASTING UP" : (velocity < -0.5 ? "FALLING" : "COASTING");
            statusColor = velocity > 0 ? Colors.Amber500 : Colors.Rose500;
        }

        renderer.DrawText(statusText, -100, -50, 3, statusColor);

        // Current altitude display (center bottom area)
        renderer.DrawText($"ALTITUDE: {altitude:F1} M", -120, -100, 3, Colors.Yellow400);
    }

    private static void DrawGraphs(Radiant.Graphics2D.Renderer2D renderer, double[][] states, double[][] controls, int currentFrame)
    {
        // Graph panel dimensions and position (top of screen)
        const float GraphPanelX = 200.0f;
        const float GraphPanelY = 50.0f;  // Moved to top (positive Y is up)
        const float GraphWidth = 250.0f;
        const float GraphHeight = 60.0f;
        const float GraphSpacing = 70.0f;

        // Draw 4 graphs: altitude, velocity, mass, thrust
        // No override - use current trajectory's max for altitude graph
        DrawSingleGraph(renderer, states, 0, "ALTITUDE (M)", GraphPanelX, GraphPanelY, GraphWidth, GraphHeight, currentFrame, Colors.Purple400);
        DrawSingleGraph(renderer, states, 1, "VELOCITY (M/S)", GraphPanelX, GraphPanelY + GraphSpacing, GraphWidth, GraphHeight, currentFrame, Colors.Cyan400);
        DrawSingleGraph(renderer, states, 2, "MASS (KG)", GraphPanelX, GraphPanelY + 2 * GraphSpacing, GraphWidth, GraphHeight, currentFrame, Colors.Emerald400);
        DrawControlGraph(renderer, controls, "THRUST (N)", GraphPanelX, GraphPanelY + 3 * GraphSpacing, GraphWidth, GraphHeight, currentFrame, Colors.Rose400);
    }

    private static void DrawSingleGraph(Radiant.Graphics2D.Renderer2D renderer, double[][] data, int stateIndex, string label, float x, float y, float width, float height, int currentFrame, Vector4 color, double? overrideMax = null)
    {
        if (data.Length < 2)
        {
            return;
        }

        // Draw label
        renderer.DrawText(label, x - 10, y - 15, 1.5f, Colors.Gray400);

        // Draw graph background
        renderer.DrawRectangleFilled(x, y, width, height, new Vector4(0.1f, 0.1f, 0.1f, 0.8f));
        renderer.DrawRectangleOutline(x, y, width, height, Colors.Gray600);

        // Find min/max for scaling
        var minVal = double.MaxValue;
        var maxVal = double.MinValue;
        for (var i = 0; i < data.Length; i++)
        {
            var val = data[i][stateIndex];
            if (val < minVal) minVal = val;
            if (val > maxVal) maxVal = val;
        }

        // Use override max if provided (for stable scaling across iterations)
        if (overrideMax.HasValue && overrideMax.Value > maxVal)
        {
            maxVal = overrideMax.Value;
        }

        // Add some padding to the range
        var range = maxVal - minVal;
        if (range < 1e-6) range = 1.0; // Avoid division by zero
        minVal -= range * 0.1;
        maxVal += range * 0.1;
        range = maxVal - minVal;

        // Draw min/max labels (in Radiant: +Y is up, so max is at top)
        renderer.DrawText($"{maxVal:F2}", x - 35, y + height, 1.2f, Colors.Gray500);
        renderer.DrawText($"{minVal:F2}", x - 35, y, 1.2f, Colors.Gray500);

        // Draw the graph line
        for (var i = 1; i < data.Length; i++)
        {
            var val1 = data[i - 1][stateIndex];
            var val2 = data[i][stateIndex];

            // Normalize to graph coordinates
            // In Radiant: +Y is up, so higher values = higher Y position
            var x1 = x + (float)(i - 1) / (data.Length - 1) * width;
            var x2 = x + (float)i / (data.Length - 1) * width;
            var y1 = y + (float)((val1 - minVal) / range) * height;
            var y2 = y + (float)((val2 - minVal) / range) * height;

            // Dim the line if it's past current frame
            var lineColor = i <= currentFrame ? color : new Vector4(color.X * 0.3f, color.Y * 0.3f, color.Z * 0.3f, 0.5f);

            renderer.DrawLine(new Vector2(x1, y1), new Vector2(x2, y2), lineColor);
        }

        // Draw current position marker
        if (currentFrame < data.Length)
        {
            var currentVal = data[currentFrame][stateIndex];
            var markerX = x + (float)currentFrame / (data.Length - 1) * width;
            var markerY = y + (float)((currentVal - minVal) / range) * height;
            renderer.DrawCircleFilled(markerX, markerY, 3, color, 8);
        }
    }

    private static void DrawControlGraph(Radiant.Graphics2D.Renderer2D renderer, double[][] controls, string label, float x, float y, float width, float height, int currentFrame, Vector4 color)
    {
        if (controls.Length < 2)
        {
            return;
        }

        // Draw label
        renderer.DrawText(label, x - 10, y - 15, 1.5f, Colors.Gray400);

        // Draw graph background
        renderer.DrawRectangleFilled(x, y, width, height, new Vector4(0.1f, 0.1f, 0.1f, 0.8f));
        renderer.DrawRectangleOutline(x, y, width, height, Colors.Gray600);

        // Find min/max for scaling (control is 1D array)
        var minVal = double.MaxValue;
        var maxVal = double.MinValue;
        for (var i = 0; i < controls.Length; i++)
        {
            var val = controls[i][0];
            if (val < minVal) minVal = val;
            if (val > maxVal) maxVal = val;
        }

        // Add some padding, ensure we show zero
        minVal = Math.Min(minVal, 0.0);
        var range = maxVal - minVal;
        if (range < 1e-6) range = 1.0;
        minVal -= range * 0.1;
        maxVal += range * 0.1;
        range = maxVal - minVal;

        // Draw min/max labels (in Radiant: +Y is up, so max is at top)
        renderer.DrawText($"{maxVal:F2}", x - 35, y + height, 1.2f, Colors.Gray500);
        renderer.DrawText($"{minVal:F2}", x - 35, y, 1.2f, Colors.Gray500);

        // Draw zero line if in range
        if (minVal <= 0 && maxVal >= 0)
        {
            var zeroY = y + (float)((0 - minVal) / range) * height;
            renderer.DrawLine(
                new Vector2(x, zeroY),
                new Vector2(x + width, zeroY),
                new Vector4(0.5f, 0.5f, 0.5f, 0.5f));
        }

        // Draw the graph line
        for (var i = 1; i < controls.Length; i++)
        {
            var val1 = controls[i - 1][0];
            var val2 = controls[i][0];

            // In Radiant: +Y is up, so higher values = higher Y position
            var x1 = x + (float)(i - 1) / (controls.Length - 1) * width;
            var x2 = x + (float)i / (controls.Length - 1) * width;
            var y1 = y + (float)((val1 - minVal) / range) * height;
            var y2 = y + (float)((val2 - minVal) / range) * height;

            var lineColor = i <= currentFrame ? color : new Vector4(color.X * 0.3f, color.Y * 0.3f, color.Z * 0.3f, 0.5f);

            renderer.DrawLine(new Vector2(x1, y1), new Vector2(x2, y2), lineColor);
        }

        // Draw current position marker
        if (currentFrame < controls.Length)
        {
            var currentVal = controls[currentFrame][0];
            var markerX = x + (float)currentFrame / (controls.Length - 1) * width;
            var markerY = y + (float)((currentVal - minVal) / range) * height;
            renderer.DrawCircleFilled(markerX, markerY, 3, color, 8);
        }
    }
}
