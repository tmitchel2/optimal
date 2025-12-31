/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Numerics;
using Radiant;

namespace OptimalCli.Problems.Brachistochrone;

/// <summary>
/// Renders a real-time graphical display of the Brachistochrone curve using Radiant.
/// </summary>
internal static class RadiantBrachistochroneVisualizer
{
    private const int WindowWidth = 800;
    private const int WindowHeight = 600;

    // Problem parameters for coordinate transformation
    private const double XFinal = 2.0;
    private const double YFinal = -2.0;
    private static readonly double DiagonalLength = Math.Sqrt(XFinal * XFinal + YFinal * YFinal);

    private static double[][]? s_currentStates;
    private static double[][]? s_currentControls;
    private static double[]? s_currentTimes;
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
    private static double[]? s_nextTimes;
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
    /// Converts (s,d) coordinates to (x,y) Cartesian coordinates.
    /// s: position along diagonal, d: perpendicular distance from diagonal
    /// </summary>
    private static (double x, double y) ConvertToCartesian(double s, double d)
    {
        // Unit vector along diagonal: e_parallel = (xFinal/L, yFinal/L)
        var eParallelX = XFinal / DiagonalLength;
        var eParallelY = YFinal / DiagonalLength;

        // Unit vector perpendicular to diagonal: e_perp = (-yFinal/L, xFinal/L)
        var ePerpX = -YFinal / DiagonalLength;
        var ePerpY = XFinal / DiagonalLength;

        // Transform: (x,y) = s * e_parallel + d * e_perp
        var x = s * eParallelX + d * ePerpX;
        var y = s * eParallelY + d * ePerpY;

        return (x, y);
    }

    /// <summary>
    /// Updates the trajectory being displayed (called from optimization progress callback).
    /// </summary>
    /// <param name="states">Array of states over time [time_index][state_vars].</param>
    /// <param name="controls">Array of controls over time [time_index][control_vars].</param>
    /// <param name="times">Array of time values at each point.</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    /// <param name="maxViolation">Maximum constraint violation.</param>
    /// <param name="constraintTolerance">Constraint tolerance for convergence.</param>
    public static void UpdateTrajectory(double[][] states, double[][] controls, double[] times, int iteration, double cost, double maxViolation, double constraintTolerance)
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
            s_nextTimes = times;
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
            app.Run("Brachistochrone Problem - Curve of Fastest Descent", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
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
        double[] times;
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
                if (s_hasNextTrajectory && s_nextStates != null && s_nextControls != null && s_nextTimes != null)
                {
                    // Start with the buffered trajectory
                    s_currentStates = s_nextStates;
                    s_currentControls = s_nextControls;
                    s_currentTimes = s_nextTimes;
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
            if (frameInSequence >= totalFrames && s_hasNextTrajectory && s_nextStates != null && s_nextControls != null && s_nextTimes != null)
            {
                // Switch to the buffered next trajectory
                s_currentStates = s_nextStates;
                s_currentControls = s_nextControls;
                s_currentTimes = s_nextTimes;
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
            times = s_currentTimes ?? new double[s_currentStates.Length];
            iteration = s_currentIteration;
            cost = s_currentCost;
            maxViolation = s_currentMaxViolation;
            constraintTolerance = s_currentConstraintTolerance;
            frameIndex = s_currentFrameIndex;
        }

        var state = states[frameIndex];
        var control = controls[frameIndex];
        var currentTime = times[frameIndex];

        var s = state[0];  // position along diagonal
        var d = state[1];  // perpendicular distance from diagonal
        var velocity = state[2];
        var theta = control[0];  // path angle in RADIANS

        // Convert (s,d) to (x,y) for visualization
        var (x, y) = ConvertToCartesian(s, d);

        // Draw the curve path
        DrawCurvePath(renderer, states, frameIndex);

        // Draw start and end markers
        DrawMarkers(renderer, states);

        // Draw the bead at current position
        DrawBead(renderer, x, y, velocity);

        // Draw information text
        DrawInformation(renderer, x, y, velocity, theta, currentTime, iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length, times[^1]);
    }

    private static void DrawCurvePath(Radiant.Graphics2D.Renderer2D renderer, double[][] states, int currentFrame)
    {
        // Find bounds for scaling - convert (s,d) to (x,y) first
        var xMin = double.MaxValue;
        var xMax = double.MinValue;
        var yMin = double.MaxValue;
        var yMax = double.MinValue;

        for (var i = 0; i < states.Length; i++)
        {
            var (x, y) = ConvertToCartesian(states[i][0], states[i][1]);
            xMin = Math.Min(xMin, x);
            xMax = Math.Max(xMax, x);
            yMin = Math.Min(yMin, y);
            yMax = Math.Max(yMax, y);
        }

        // Add padding
        var xRange = xMax - xMin;
        var yRange = yMax - yMin;
        if (xRange < 1e-6) xRange = 1.0;
        if (yRange < 1e-6) yRange = 1.0;

        var xCenter = (xMin + xMax) / 2.0;
        var yCenter = (yMin + yMax) / 2.0;

        // Calculate scale to fit in window
        const float DrawWidth = 600.0f;
        const float DrawHeight = 400.0f;
        var scaleX = (float)(DrawWidth / (xRange * 1.3));
        var scaleY = (float)(DrawHeight / (yRange * 1.3));
        var scale = Math.Min(scaleX, scaleY);

        // Draw the path trace up to current frame (animated, in color)
        for (var i = 0; i < currentFrame && i < states.Length - 1; i++)
        {
            var (x1Pos, y1Pos) = ConvertToCartesian(states[i][0], states[i][1]);
            var (x2Pos, y2Pos) = ConvertToCartesian(states[i + 1][0], states[i + 1][1]);
            var x1 = (float)((x1Pos - xCenter) * scale);
            var y1 = (float)((y1Pos - yCenter) * scale);
            var x2 = (float)((x2Pos - xCenter) * scale);
            var y2 = (float)((y2Pos - yCenter) * scale);

            // Color gradient from start (cyan) to current (purple)
            var t = (float)i / currentFrame;
            var color = new Vector4(0.2f + (0.6f * t), 0.6f - (0.4f * t), 0.8f, 0.9f);

            renderer.DrawLine(
                new Vector2(x1, y1),
                new Vector2(x2, y2),
                color);
        }

        // Draw remaining path in light gray (ghost path)
        for (var i = currentFrame; i < states.Length - 1; i++)
        {
            var (x1Pos, y1Pos) = ConvertToCartesian(states[i][0], states[i][1]);
            var (x2Pos, y2Pos) = ConvertToCartesian(states[i + 1][0], states[i + 1][1]);
            var x1 = (float)((x1Pos - xCenter) * scale);
            var y1 = (float)((y1Pos - yCenter) * scale);
            var x2 = (float)((x2Pos - xCenter) * scale);
            var y2 = (float)((y2Pos - yCenter) * scale);

            renderer.DrawLine(
                new Vector2(x1, y1),
                new Vector2(x2, y2),
                new Vector4(0.4f, 0.4f, 0.4f, 0.6f));
        }

        // Draw reference lines (straight line from start to end)
        if (states.Length >= 2)
        {
            var (startXPos, startYPos) = ConvertToCartesian(states[0][0], states[0][1]);
            var (endXPos, endYPos) = ConvertToCartesian(states[^1][0], states[^1][1]);
            var startX = (float)((startXPos - xCenter) * scale);
            var startY = (float)((startYPos - yCenter) * scale);
            var endX = (float)((endXPos - xCenter) * scale);
            var endY = (float)((endYPos - yCenter) * scale);

            renderer.DrawLine(
                new Vector2(startX, startY),
                new Vector2(endX, endY),
                new Vector4(0.3f, 0.3f, 0.3f, 0.4f));
        }
    }

    private static void DrawMarkers(Radiant.Graphics2D.Renderer2D renderer, double[][] states)
    {
        if (states.Length < 2) return;

        // Find bounds for scaling (same as DrawCurvePath) - convert (s,d) to (x,y)
        var xMin = double.MaxValue;
        var xMax = double.MinValue;
        var yMin = double.MaxValue;
        var yMax = double.MinValue;

        for (var i = 0; i < states.Length; i++)
        {
            var (sx, sy) = ConvertToCartesian(states[i][0], states[i][1]);
            xMin = Math.Min(xMin, sx);
            xMax = Math.Max(xMax, sx);
            yMin = Math.Min(yMin, sy);
            yMax = Math.Max(yMax, sy);
        }

        var xRange = xMax - xMin;
        var yRange = yMax - yMin;
        if (xRange < 1e-6) xRange = 1.0;
        if (yRange < 1e-6) yRange = 1.0;

        var xCenter = (xMin + xMax) / 2.0;
        var yCenter = (yMin + yMax) / 2.0;

        const float DrawWidth = 600.0f;
        const float DrawHeight = 400.0f;
        var scaleX = (float)(DrawWidth / (xRange * 1.3));
        var scaleY = (float)(DrawHeight / (yRange * 1.3));
        var scale = Math.Min(scaleX, scaleY);

        // Start position (green)
        var (startXPos, startYPos) = ConvertToCartesian(states[0][0], states[0][1]);
        var startX = (float)((startXPos - xCenter) * scale);
        var startY = (float)((startYPos - yCenter) * scale);
        renderer.DrawCircleFilled(startX, startY, 10, Colors.Emerald500, 24);
        renderer.DrawCircleOutline(startX, startY, 10, Colors.Emerald300, 24);
        renderer.DrawText("START", startX - 30, startY - 25, 2, Colors.Emerald400);

        // End position (red)
        var (endXPos, endYPos) = ConvertToCartesian(states[^1][0], states[^1][1]);
        var endX = (float)((endXPos - xCenter) * scale);
        var endY = (float)((endYPos - yCenter) * scale);
        renderer.DrawCircleFilled(endX, endY, 10, Colors.Rose500, 24);
        renderer.DrawCircleOutline(endX, endY, 10, Colors.Rose300, 24);
        renderer.DrawText("END", endX - 20, endY + 20, 2, Colors.Rose400);
    }

    private static void DrawBead(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double velocity)
    {
        // Find bounds for scaling (same as DrawCurvePath) - convert (s,d) to (x,y)
        if (s_currentStates == null) return;

        var xMin = double.MaxValue;
        var xMax = double.MinValue;
        var yMin = double.MaxValue;
        var yMax = double.MinValue;

        for (var i = 0; i < s_currentStates.Length; i++)
        {
            var (sx, sy) = ConvertToCartesian(s_currentStates[i][0], s_currentStates[i][1]);
            xMin = Math.Min(xMin, sx);
            xMax = Math.Max(xMax, sx);
            yMin = Math.Min(yMin, sy);
            yMax = Math.Max(yMax, sy);
        }

        var xRange = xMax - xMin;
        var yRange = yMax - yMin;
        if (xRange < 1e-6) xRange = 1.0;
        if (yRange < 1e-6) yRange = 1.0;

        var xCenter = (xMin + xMax) / 2.0;
        var yCenter = (yMin + yMax) / 2.0;

        const float DrawWidth = 600.0f;
        const float DrawHeight = 400.0f;
        var scaleX = (float)(DrawWidth / (xRange * 1.3));
        var scaleY = (float)(DrawHeight / (yRange * 1.3));
        var scale = Math.Min(scaleX, scaleY);

        var beadX = (float)((x - xCenter) * scale);
        var beadY = (float)((y - yCenter) * scale);

        // Draw bead with glow effect (velocity-based size)
        var beadRadius = 15.0f + (float)(velocity * 2.0f);
        beadRadius = Math.Min(beadRadius, 30.0f);

        // Outer glow
        renderer.DrawCircleFilled(beadX, beadY, beadRadius + 5, new Vector4(0.4f, 0.7f, 1.0f, 0.3f), 32);

        // Main bead
        renderer.DrawCircleFilled(beadX, beadY, beadRadius, Colors.Cyan400, 32);

        // Highlight
        renderer.DrawCircleFilled(beadX - 5, beadY - 5, beadRadius * 0.4f, Colors.Cyan200, 16);

        // Outline
        renderer.DrawCircleOutline(beadX, beadY, beadRadius, Colors.Cyan300, 32);

        // Draw velocity vector (arrow showing direction and magnitude)
        if (velocity > 0.1)
        {
            // For Brachistochrone, we need to determine direction from states
            var currentFrame = s_currentFrameIndex;
            if (currentFrame < s_currentStates.Length - 1)
            {
                var (nextXPos, nextYPos) = ConvertToCartesian(s_currentStates[currentFrame + 1][0], s_currentStates[currentFrame + 1][1]);
                var nextX = (float)((nextXPos - xCenter) * scale);
                var nextY = (float)((nextYPos - yCenter) * scale);

                var dx = nextX - beadX;
                var dy = nextY - beadY;
                var dist = (float)Math.Sqrt(dx * dx + dy * dy);

                if (dist > 0.1f)
                {
                    dx /= dist;
                    dy /= dist;

                    var arrowLength = (float)Math.Min(velocity * 10.0, 50.0);
                    var arrowEndX = beadX + (dx * arrowLength);
                    var arrowEndY = beadY + (dy * arrowLength);

                    // Arrow shaft
                    renderer.DrawLine(
                        new Vector2(beadX, beadY),
                        new Vector2(arrowEndX, arrowEndY),
                        Colors.Yellow400);

                    // Arrow head
                    var headSize = 8.0f;
                    var perpX = -dy;
                    var perpY = dx;

                    renderer.DrawLine(
                        new Vector2(arrowEndX, arrowEndY),
                        new Vector2(arrowEndX - (dx * headSize) + (perpX * headSize * 0.5f), arrowEndY - (dy * headSize) + (perpY * headSize * 0.5f)),
                        Colors.Yellow400);
                    renderer.DrawLine(
                        new Vector2(arrowEndX, arrowEndY),
                        new Vector2(arrowEndX - (dx * headSize) - (perpX * headSize * 0.5f), arrowEndY - (dy * headSize) - (perpY * headSize * 0.5f)),
                        Colors.Yellow400);
                }
            }
        }
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double velocity, double theta, double currentTime, int iteration, double cost, double maxViolation, double constraintTolerance, int frameIndex, int totalFrames, double finalTime)
    {
        // Draw text information at the top
        const float TopY = -270.0f;

        // Iteration and cost (descent time)
        renderer.DrawText($"ITERATION: {iteration}", -380, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"ELAPSED: {currentTime:F3} S / {finalTime:F3} S", -380, TopY + 20, 2, Colors.Sky400);

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

        // Position (display both Cartesian and diagonal coordinates)
        renderer.DrawText($"X: {x:F2} M  Y: {y:F2} M", 200, TopY + 20, 2, Colors.Purple400);

        // Velocity
        renderer.DrawText($"VELOCITY: {velocity:F2} M-S", 200, TopY + 40, 2, Colors.Cyan400);

        // Path angle in degrees (now relative to diagonal, not horizontal)
        var angleDegrees = theta * 180.0 / Math.PI;  // Convert radians to degrees for display
        renderer.DrawText($"PATH ANGLE: {angleDegrees:F1} DEG (rel diagonal)", 150, TopY + 60, 2, Colors.Rose400);

        // Title at bottom
        renderer.DrawText("BRACHISTOCHRONE - FASTEST DESCENT", -200, 250, 2, Colors.Gray400);

        // Frame progress bar
        if (totalFrames > 0)
        {
            const float FrameBarWidth = 600.0f;
            const float FrameBarHeight = 8.0f;
            const float FrameBarX = -FrameBarWidth / 2;
            const float FrameBarY = 270.0f;

            renderer.DrawRectangleFilled(FrameBarX, FrameBarY, FrameBarWidth, FrameBarHeight, Colors.Slate700);
            var frameProgress = (float)frameIndex / totalFrames * FrameBarWidth;
            renderer.DrawRectangleFilled(FrameBarX, FrameBarY, frameProgress, FrameBarHeight, Colors.Cyan600);
        }
    }
}
