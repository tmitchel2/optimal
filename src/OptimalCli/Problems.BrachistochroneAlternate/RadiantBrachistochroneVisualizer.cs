/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Numerics;
using Radiant;

namespace OptimalCli.Problems.BrachistochroneAlternate;

/// <summary>
/// Renders a real-time graphical display of the Brachistochrone curve using Radiant.
///
/// State format (arc-length parameterization):
///   [v, n, alpha, t] where:
///   - v: Speed (m/s)
///   - n: Perpendicular distance from reference line (m, positive = below line)
///   - alpha: Heading angle relative to reference line (rad)
///   - t: Elapsed time (s)
///
/// Control format:
///   [k] where k is path curvature (rad/m)
///
/// Independent variable is s (horizontal distance), reconstructed from index.
/// </summary>
internal static class RadiantBrachistochroneVisualizer
{
    private const int WindowWidth = 1000;
    private const int WindowHeight = 700;
    private const float ScaleX = 35.0f; // Pixels per meter horizontal
    private const float ScaleY = 35.0f; // Pixels per meter vertical

    // Reference line geometry from dynamics
    private static readonly double ThetaRef = BrachistochroneAlternateDynamics.ThetaRef;
    private static readonly double STotal = BrachistochroneAlternateDynamics.STotal;

    // State indices
    private const int IdxV = 0;
    private const int IdxN = 1;
    private const int IdxAlpha = 2;
    private const int IdxT = 3;

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

    /// <summary>
    /// Updates the trajectory being displayed (called from optimization progress callback).
    /// </summary>
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

            Console.WriteLine($"[VIZ] Buffered Iter {iteration}: {states.Length} frames, cost={cost:F4}s, violation={maxViolation:E2}/{constraintTolerance:E2}");
        }
    }

    /// <summary>
    /// Runs the visualization window and blocks until closed.
    /// </summary>
    /// <param name="cancellationTokenSource">The cancellation token source to signal when the window is closed.</param>
    public static void RunVisualizationWindow(CancellationTokenSource cancellationTokenSource)
    {
        try
        {
            using var app = new RadiantApplication();
            app.Run("Brachistochrone Problem", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
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
                    s_currentFrameIndex = 0;
                    s_animationStartTime = DateTime.Now;
                    s_hasNextTrajectory = false;

                    Console.WriteLine($"[VIZ] Started displaying Iter {s_currentIteration}");
                }
                else
                {
                    renderer.DrawText("WAITING FOR OPTIMIZATION...", -300, 0, 3, Colors.Gray400);
                    return;
                }
            }

            var elapsed = (DateTime.Now - s_animationStartTime).TotalMilliseconds;
            const double FrameDuration = 50.0; // 50ms per frame
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

        // Draw the curve path
        DrawCurvePath(renderer, states);

        // Draw start and end points
        DrawStartEndPoints(renderer, states);

        // Draw the bead at current frame
        DrawBead(renderer, states, frameIndex);

        // Draw information
        DrawInformation(renderer, states, controls, iteration, cost, maxViolation, constraintTolerance, frameIndex);
    }

    /// <summary>
    /// Converts state index to (x, y) screen coordinates.
    /// Transforms from rotated (s, n) coordinates to Cartesian (x, y_down) for display.
    ///
    /// Coordinate transformation:
    ///   x = s * cos(ThetaRef) - n * sin(ThetaRef)
    ///   y_down = s * sin(ThetaRef) + n * cos(ThetaRef)
    /// </summary>
    private static (float screenX, float screenY) GetScreenPosition(double[][] states, int index)
    {
        var numPoints = states.Length;
        var s = (double)index / (numPoints - 1) * STotal; // Distance along reference line
        var n = states[index][IdxN]; // Perpendicular distance from reference line

        // Convert from rotated (s, n) to Cartesian (x, y_down)
        var x = s * Math.Cos(ThetaRef) - n * Math.Sin(ThetaRef);
        var yDown = s * Math.Sin(ThetaRef) + n * Math.Cos(ThetaRef);

        // Convert to screen coordinates
        var screenX = (float)(x * ScaleX) - 450.0f;
        var screenY = (float)(-yDown * ScaleY) + 100.0f; // Negate because Radiant y-axis points up

        return (screenX, screenY);
    }

    private static void DrawCurvePath(Radiant.Graphics2D.Renderer2D renderer, double[][] states)
    {
        // Draw the trajectory path
        for (var i = 0; i < states.Length - 1; i++)
        {
            var (x1, y1) = GetScreenPosition(states, i);
            var (x2, y2) = GetScreenPosition(states, i + 1);

            renderer.DrawLine(
                new Vector2(x1, y1),
                new Vector2(x2, y2),
                new Vector4(0.4f, 0.6f, 0.9f, 0.8f));
        }
    }

    private static void DrawStartEndPoints(Radiant.Graphics2D.Renderer2D renderer, double[][] states)
    {
        // Start point (green)
        var (startX, startY) = GetScreenPosition(states, 0);
        renderer.DrawCircleFilled(startX, startY, 8, Colors.Emerald500, 16);
        renderer.DrawText("START", startX - 30, startY + 25, 1.5f, Colors.Emerald400);

        // End point (red)
        var (endX, endY) = GetScreenPosition(states, states.Length - 1);
        renderer.DrawCircleFilled(endX, endY, 8, Colors.Rose500, 16);
        renderer.DrawText("END", endX - 20, endY - 15, 1.5f, Colors.Rose400);

        // Draw straight line reference (dashed)
        var segments = 20;
        for (var i = 0; i < segments; i++)
        {
            if (i % 2 == 0)
            {
                var t1 = i / (float)segments;
                var t2 = (i + 1) / (float)segments;
                var x1 = startX + (endX - startX) * t1;
                var y1 = startY + (endY - startY) * t1;
                var x2 = startX + (endX - startX) * t2;
                var y2 = startY + (endY - startY) * t2;
                renderer.DrawLine(new Vector2(x1, y1), new Vector2(x2, y2), new Vector4(0.3f, 0.3f, 0.3f, 0.5f));
            }
        }
    }

    private static void DrawBead(Radiant.Graphics2D.Renderer2D renderer, double[][] states, int frameIndex)
    {
        var (x, y) = GetScreenPosition(states, frameIndex);
        var state = states[frameIndex];
        var v = state[IdxV];
        var alpha = state[IdxAlpha];

        // Draw velocity vector
        // alpha is relative to reference line, so actual world angle is alpha + ThetaRef
        var worldAlpha = alpha + ThetaRef;
        var velocityScale = 10.0f;
        var vx = (float)(v * Math.Cos(worldAlpha)) * velocityScale;
        var vy = (float)(-v * Math.Sin(worldAlpha)) * velocityScale; // Negate for screen coords
        renderer.DrawLine(
            new Vector2(x, y),
            new Vector2(x + vx, y + vy),
            new Vector4(1.0f, 0.8f, 0.2f, 0.8f));

        // Draw bead
        renderer.DrawCircleFilled(x, y, 12, Colors.Sky400, 24);
        renderer.DrawCircleOutline(x, y, 12, Colors.Sky200, 24);
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer, double[][] states, double[][] controls, int iteration, double cost, double maxViolation, double constraintTolerance, int frameIndex)
    {
        const float TopY = -320.0f;

        // Iteration and cost
        renderer.DrawText($"ITERATION: {iteration}", -480, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"TIME: {cost:F4} s", -480, TopY + 20, 2, Colors.Sky400);

        // Convergence
        var convergenceRatio = constraintTolerance > 0 ? maxViolation / constraintTolerance : 0.0;
        var isConverged = maxViolation < constraintTolerance;
        var convergenceColor = isConverged ? Colors.Emerald500 : (convergenceRatio < 10.0 ? Colors.Amber500 : Colors.Rose500);
        renderer.DrawText($"CONVERGENCE: {maxViolation:E2}/{constraintTolerance:E2}", -480, TopY + 40, 2, convergenceColor);

        // Progress bar
        const float ConvBarWidth = 200.0f;
        const float ConvBarHeight = 8.0f;
        const float ConvBarX = -480.0f;
        const float ConvBarY = TopY + 60.0f;

        renderer.DrawRectangleFilled(ConvBarX, ConvBarY, ConvBarWidth, ConvBarHeight, Colors.Slate700);
        var progress = Math.Min(1.0, Math.Max(0.0, 1.0 - Math.Log10(Math.Max(0.1, convergenceRatio)) / 2.0));
        var progressWidth = (float)(progress * ConvBarWidth);
        renderer.DrawRectangleFilled(ConvBarX, ConvBarY, progressWidth, ConvBarHeight, convergenceColor);

        // Current state
        var state = states[frameIndex];
        var control = controls[frameIndex];
        var s = (double)frameIndex / (states.Length - 1) * STotal;

        renderer.DrawText($"FRAME: {frameIndex + 1}/{states.Length}", 280, TopY, 2, Colors.Amber400);
        renderer.DrawText($"POSITION: s={s:F2}m (along line), n={state[IdxN]:F2}m (perp)", 280, TopY + 20, 2, Colors.Purple400);
        renderer.DrawText($"VELOCITY: {state[IdxV]:F2} m/s", 280, TopY + 40, 2, Colors.Rose400);
        renderer.DrawText($"ANGLE: {state[IdxAlpha] * 180 / Math.PI:F1} deg", 280, TopY + 60, 2, Colors.Cyan400);
        renderer.DrawText($"TIME: {state[IdxT]:F3} s", 280, TopY + 80, 2, Colors.Lime400);
        renderer.DrawText($"CURVATURE: {control[0]:F4} rad/m", 280, TopY + 100, 2, Colors.Orange400);

        // Animation progress bar
        const float FrameBarHeight = 8.0f;
        const float FrameBarY = 310.0f;
        const float FrameBarX = -480.0f;
        const float FrameBarWidth = 960.0f;
        renderer.DrawRectangleFilled(FrameBarX, FrameBarY, FrameBarWidth, FrameBarHeight, Colors.Slate700);
        if (states.Length > 0)
        {
            var frameProgress = (float)frameIndex / states.Length * FrameBarWidth;
            renderer.DrawRectangleFilled(FrameBarX, FrameBarY, frameProgress, FrameBarHeight, Colors.Sky600);
        }
    }
}
