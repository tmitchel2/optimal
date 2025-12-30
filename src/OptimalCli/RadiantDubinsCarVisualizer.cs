/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Numerics;
using Radiant;

namespace OptimalCli;

/// <summary>
/// Renders a real-time graphical display of a Dubins car path using Radiant.
/// </summary>
internal static class RadiantDubinsCarVisualizer
{
    private const int WindowWidth = 800;
    private const int WindowHeight = 600;

    private static double[][]? s_currentStates;
    private static double[][]? s_currentControls;
    private static int s_currentIteration;
    private static double s_currentCost;
    private static double s_currentMaxViolation;
    private static double s_currentConstraintTolerance;
    private static int s_currentFrameIndex;
    private static DateTime s_animationStartTime;
    private static readonly object s_lock = new();

    // Buffered next trajectory (waiting to be displayed)
    private static double[][]? s_nextStates;
    private static double[][]? s_nextControls;
    private static int s_nextIteration;
    private static double s_nextCost;
    private static double s_nextMaxViolation;
    private static double s_nextConstraintTolerance;
    private static bool s_hasNextTrajectory;

    // Cancellation token for stopping optimization when window closes
    private static CancellationTokenSource? s_cancellationTokenSource;

    /// <summary>
    /// Gets the cancellation token that signals when the visualization window is closed.
    /// </summary>
    public static CancellationToken CancellationToken => s_cancellationTokenSource?.Token ?? CancellationToken.None;

    /// <summary>
    /// Updates the trajectory being displayed (called from optimization progress callback).
    /// </summary>
    /// <param name="states">Array of states over time [time_index][state_vars].</param>
    /// <param name="controls">Array of controls over time [time_index][control_vars].</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    /// <param name="maxViolation">Maximum constraint violation.</param>
    /// <param name="constraintTolerance">Constraint tolerance for convergence.</param>
    public static void UpdateTrajectory(double[][] states, double[][] controls, int iteration, double cost, double maxViolation, double constraintTolerance)
    {
        if (states.Length == 0 || controls.Length == 0)
        {
            return;
        }

        lock (s_lock)
        {
            // Buffer this update - it will be applied when the current animation completes
            s_nextStates = states;
            s_nextControls = controls;
            s_nextIteration = iteration;
            s_nextCost = cost;
            s_nextMaxViolation = maxViolation;
            s_nextConstraintTolerance = constraintTolerance;
            s_hasNextTrajectory = true;

            // Debug output
            Console.WriteLine($"[VIZ] Buffered Iter {iteration}: {states.Length} frames, cost={cost:F4}, violation={maxViolation:E2}/{constraintTolerance:E2}");
        }
    }

    /// <summary>
    /// Runs the visualization window and blocks until closed.
    /// Call this on the main thread while the optimizer runs in a background task.
    /// </summary>
    public static void RunVisualizationWindow()
    {
        // Create cancellation token source for this visualization session
        s_cancellationTokenSource = new CancellationTokenSource();

        try
        {
            using var app = new RadiantApplication();
            app.Run("Dubins Car Path Optimization", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
        }
        finally
        {
            // Signal cancellation when window closes
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
                // No current trajectory - check if there's a buffered one
                if (s_hasNextTrajectory && s_nextStates != null && s_nextControls != null)
                {
                    // Start with the buffered trajectory
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
                    // Still waiting
                    renderer.DrawText("WAITING FOR OPTIMIZATION...", -200, 0, 3, Colors.Gray400);
                    return;
                }
            }

            // Calculate elapsed time and determine current frame
            var elapsed = (DateTime.Now - s_animationStartTime).TotalMilliseconds;
            const double FrameDuration = 100.0;
            var totalFrames = s_currentStates.Length;
            var frameInSequence = (int)(elapsed / FrameDuration);

            // Check if we've completed at least one full loop
            if (frameInSequence >= totalFrames && s_hasNextTrajectory && s_nextStates != null && s_nextControls != null)
            {
                // Switch to the buffered next trajectory
                s_currentStates = s_nextStates;
                s_currentControls = s_nextControls;
                s_currentIteration = s_nextIteration;
                s_currentCost = s_nextCost;
                s_currentMaxViolation = s_nextMaxViolation;
                s_currentConstraintTolerance = s_nextConstraintTolerance;
                s_currentFrameIndex = 0;
                s_animationStartTime = DateTime.Now;
                s_hasNextTrajectory = false;

                Console.WriteLine($"[VIZ] Switched to Iter {s_currentIteration} after completing previous animation");

                // Reset for new trajectory
                frameInSequence = 0;
                totalFrames = s_currentStates.Length;
            }

            // Loop the animation continuously
            s_currentFrameIndex = frameInSequence % totalFrames;

            states = s_currentStates;
            controls = s_currentControls;
            iteration = s_currentIteration;
            cost = s_currentCost;
            maxViolation = s_currentMaxViolation;
            constraintTolerance = s_currentConstraintTolerance;
            frameIndex = s_currentFrameIndex;
        }

        // Extract current state and control
        var state = states[frameIndex];
        var control = controls[frameIndex];

        var x = state[0];
        var y = state[1];
        var heading = state[2];
        var turnRate = control[0];

        // Draw the path trace
        DrawPathTrace(renderer, states, frameIndex);

        // Draw the car
        DrawCar(renderer, x, y, heading);

        // Draw start and goal markers
        DrawMarkers(renderer, states);

        // Draw information text
        DrawInformation(renderer, x, y, heading, turnRate, iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length);
    }

    private static void DrawPathTrace(Radiant.Graphics2D.Renderer2D renderer, double[][] states, int currentFrame)
    {
        // Find bounds for scaling
        var xMin = double.MaxValue;
        var xMax = double.MinValue;
        var yMin = double.MaxValue;
        var yMax = double.MinValue;

        for (var i = 0; i < states.Length; i++)
        {
            var x = states[i][0];
            var y = states[i][1];
            xMin = Math.Min(xMin, x);
            xMax = Math.Max(xMax, x);
            yMin = Math.Min(yMin, y);
            yMax = Math.Max(yMax, y);
        }

        // Add padding and ensure equal scaling
        var xRange = xMax - xMin;
        var yRange = yMax - yMin;
        if (xRange < 1e-6) xRange = 1.0;
        if (yRange < 1e-6) yRange = 1.0;

        var maxRange = Math.Max(xRange, yRange);
        var xCenter = (xMin + xMax) / 2.0;
        var yCenter = (yMin + yMax) / 2.0;

        var paddedRange = maxRange * 1.2;

        // Calculate scale to fit in window
        const float DrawSize = 500.0f;
        var scale = (float)(DrawSize / paddedRange);

        // Draw the path up to current frame
        for (var i = 0; i < currentFrame; i++)
        {
            var x1 = (float)((states[i][0] - xCenter) * scale);
            var y1 = (float)((states[i][1] - yCenter) * scale);
            var x2 = (float)((states[i + 1][0] - xCenter) * scale);
            var y2 = (float)((states[i + 1][1] - yCenter) * scale);

            // Color gradient from start (green) to current (cyan)
            var t = (float)i / currentFrame;
            var color = new Vector4(0.3f + (0.3f * t), 0.8f - (0.3f * t), 0.8f, 0.8f);

            renderer.DrawLine(
                new Vector2(x1, -y1),  // Negate Y for screen coords
                new Vector2(x2, -y2),
                color);
        }

        // Draw remaining path in gray
        for (var i = currentFrame; i < states.Length - 1; i++)
        {
            var x1 = (float)((states[i][0] - xCenter) * scale);
            var y1 = (float)((states[i][1] - yCenter) * scale);
            var x2 = (float)((states[i + 1][0] - xCenter) * scale);
            var y2 = (float)((states[i + 1][1] - yCenter) * scale);

            renderer.DrawLine(
                new Vector2(x1, -y1),
                new Vector2(x2, -y2),
                new Vector4(0.3f, 0.3f, 0.3f, 0.5f));
        }
    }

    private static void DrawCar(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double heading)
    {
        // Find bounds for scaling (recompute from current states)
        var xMin = double.MaxValue;
        var xMax = double.MinValue;
        var yMin = double.MaxValue;
        var yMax = double.MinValue;

        if (s_currentStates != null)
        {
            for (var i = 0; i < s_currentStates.Length; i++)
            {
                var sx = s_currentStates[i][0];
                var sy = s_currentStates[i][1];
                xMin = Math.Min(xMin, sx);
                xMax = Math.Max(xMax, sx);
                yMin = Math.Min(yMin, sy);
                yMax = Math.Max(yMax, sy);
            }
        }

        var xRange = xMax - xMin;
        var yRange = yMax - yMin;
        if (xRange < 1e-6) xRange = 1.0;
        if (yRange < 1e-6) yRange = 1.0;

        var maxRange = Math.Max(xRange, yRange);
        var xCenter = (xMin + xMax) / 2.0;
        var yCenter = (yMin + yMax) / 2.0;

        var paddedRange = maxRange * 1.2;
        const float DrawSize = 500.0f;
        var scale = (float)(DrawSize / paddedRange);

        var carX = (float)((x - xCenter) * scale);
        var carY = -(float)((y - yCenter) * scale);

        // Draw car as a triangle pointing in heading direction
        const float CarSize = 15.0f;
        var frontX = carX + (float)(CarSize * Math.Cos(heading));
        var frontY = carY - (float)(CarSize * Math.Sin(heading));  // Negate for screen coords

        var angle1 = heading + (2.5 * Math.PI / 3.0);
        var angle2 = heading - (2.5 * Math.PI / 3.0);

        var backLeft = carX + (float)(CarSize * 0.7 * Math.Cos(angle1));
        var backLeftY = carY - (float)(CarSize * 0.7 * Math.Sin(angle1));

        var backRightX = carX + (float)(CarSize * 0.7 * Math.Cos(angle2));
        var backRightY = carY - (float)(CarSize * 0.7 * Math.Sin(angle2));

        // Draw triangle using lines (Radiant doesn't have triangle primitives)
        renderer.DrawLine(new Vector2(frontX, frontY), new Vector2(backLeft, backLeftY), Colors.Cyan500);
        renderer.DrawLine(new Vector2(backLeft, backLeftY), new Vector2(backRightX, backRightY), Colors.Cyan500);
        renderer.DrawLine(new Vector2(backRightX, backRightY), new Vector2(frontX, frontY), Colors.Cyan500);

        // Draw center circle to make it more visible
        renderer.DrawCircleFilled(carX, carY, 8, Colors.Cyan300, 16);
    }

    private static void DrawMarkers(Radiant.Graphics2D.Renderer2D renderer, double[][] states)
    {
        if (states.Length < 2) return;

        // Find bounds for scaling
        var xMin = double.MaxValue;
        var xMax = double.MinValue;
        var yMin = double.MaxValue;
        var yMax = double.MinValue;

        for (var i = 0; i < states.Length; i++)
        {
            var sx = states[i][0];
            var sy = states[i][1];
            xMin = Math.Min(xMin, sx);
            xMax = Math.Max(xMax, sx);
            yMin = Math.Min(yMin, sy);
            yMax = Math.Max(yMax, sy);
        }

        var xRange = xMax - xMin;
        var yRange = yMax - yMin;
        if (xRange < 1e-6) xRange = 1.0;
        if (yRange < 1e-6) yRange = 1.0;

        var maxRange = Math.Max(xRange, yRange);
        var xCenter = (xMin + xMax) / 2.0;
        var yCenter = (yMin + yMax) / 2.0;

        var paddedRange = maxRange * 1.2;
        const float DrawSize = 500.0f;
        var scale = (float)(DrawSize / paddedRange);

        // Start position (green)
        var startX = (float)((states[0][0] - xCenter) * scale);
        var startY = -(float)((states[0][1] - yCenter) * scale);
        renderer.DrawCircleFilled(startX, startY, 12, Colors.Emerald500, 24);
        renderer.DrawCircleOutline(startX, startY, 12, Colors.Emerald300, 24);
        renderer.DrawText("START", startX - 30, startY - 25, 2, Colors.Emerald400);

        // Goal position (red)
        var goalX = (float)((states[^1][0] - xCenter) * scale);
        var goalY = -(float)((states[^1][1] - yCenter) * scale);
        renderer.DrawCircleFilled(goalX, goalY, 12, Colors.Rose500, 24);
        renderer.DrawCircleOutline(goalX, goalY, 12, Colors.Rose300, 24);
        renderer.DrawText("GOAL", goalX - 25, goalY + 20, 2, Colors.Rose400);
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double heading, double turnRate, int iteration, double cost, double maxViolation, double constraintTolerance, int frameIndex, int totalFrames)
    {
        // Draw text information at the top
        const float TopY = -270.0f;

        // Iteration and cost
        renderer.DrawText($"ITERATION: {iteration}", -380, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"COST: {cost:F4}", -380, TopY + 20, 2, Colors.Sky400);

        // Convergence progress
        var convergenceRatio = constraintTolerance > 0 ? maxViolation / constraintTolerance : 0.0;
        var isConverged = maxViolation < constraintTolerance;
        var convergenceColor = isConverged ? Colors.Emerald500 : (convergenceRatio < 10.0 ? Colors.Amber500 : Colors.Rose500);
        renderer.DrawText($"CONVERGENCE: {maxViolation:E2}/{constraintTolerance:E2}", -380, TopY + 40, 2, convergenceColor);

        // Draw convergence progress bar
        const float ConvBarWidth = 200.0f;
        const float ConvBarHeight = 8.0f;
        const float ConvBarX = -380.0f;
        const float ConvBarY = TopY + 60.0f;

        // Background
        renderer.DrawRectangleFilled(ConvBarX, ConvBarY, ConvBarWidth, ConvBarHeight, Colors.Slate700);

        // Progress (clamp to 0-1, log scale for better visibility)
        var progress = Math.Min(1.0, Math.Max(0.0, 1.0 - Math.Log10(Math.Max(0.1, convergenceRatio)) / 2.0));
        var progressWidth = (float)(progress * ConvBarWidth);
        renderer.DrawRectangleFilled(ConvBarX, ConvBarY, progressWidth, ConvBarHeight, convergenceColor);

        // Frame info
        renderer.DrawText($"FRAME: {frameIndex + 1}/{totalFrames}", 200, TopY, 2, Colors.Amber400);

        // Position
        renderer.DrawText($"X: {x:F2}  Y: {y:F2}", 200, TopY + 20, 2, Colors.Purple400);

        // Heading in degrees
        var headingDegrees = heading * 180.0 / Math.PI;
        renderer.DrawText($"HEADING: {headingDegrees:F1} DEG", 200, TopY + 40, 2, Colors.Cyan400);

        // Turn rate
        renderer.DrawText($"TURN RATE: {turnRate:F2} RAD-S", 200, TopY + 60, 2, Colors.Rose400);
    }
}
