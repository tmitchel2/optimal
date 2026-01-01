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
    private const int WindowWidth = 1000;
    private const int WindowHeight = 700;
    private const float ScaleX = 35.0f; // Pixels per meter horizontal
    private const float ScaleY = 35.0f; // Pixels per meter vertical

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
    public static void RunVisualizationWindow()
    {
        s_cancellationTokenSource = new CancellationTokenSource();

        try
        {
            using var app = new RadiantApplication();
            app.Run("Brachistochrone Problem", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
        }
        finally
        {
            if (s_cancellationTokenSource != null && !s_cancellationTokenSource.IsCancellationRequested)
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
        DrawBead(renderer, states[frameIndex]);

        // Draw information
        DrawInformation(renderer, states, controls, iteration, cost, maxViolation, constraintTolerance, frameIndex);
    }

    private static void DrawCurvePath(Radiant.Graphics2D.Renderer2D renderer, double[][] states)
    {
        // Draw the trajectory path
        // Both state and Radiant use left-hand coordinates (y-axis up), so no flipping needed
        for (var i = 0; i < states.Length - 1; i++)
        {
            var x1 = (float)(states[i][0] * ScaleX) - 450.0f;
            var y1 = (float)(states[i][1] * ScaleY) - 100.0f;
            var x2 = (float)(states[i + 1][0] * ScaleX) - 450.0f;
            var y2 = (float)(states[i + 1][1] * ScaleY) - 100.0f;

            renderer.DrawLine(
                new Vector2(x1, y1),
                new Vector2(x2, y2),
                new Vector4(0.4f, 0.6f, 0.9f, 0.8f));
        }
    }

    private static void DrawStartEndPoints(Radiant.Graphics2D.Renderer2D renderer, double[][] states)
    {
        // Start point (green)
        var startX = (float)(states[0][0] * ScaleX) - 450.0f;
        var startY = (float)(states[0][1] * ScaleY) - 100.0f;
        renderer.DrawCircleFilled(startX, startY, 8, Colors.Emerald500, 16);
        renderer.DrawText("START", startX - 30, startY + 25, 1.5f, Colors.Emerald400);

        // End point (red)
        var endX = (float)(states[^1][0] * ScaleX) - 450.0f;
        var endY = (float)(states[^1][1] * ScaleY) - 100.0f;
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

    private static void DrawBead(Radiant.Graphics2D.Renderer2D renderer, double[] state)
    {
        var x = (float)(state[0] * ScaleX) - 450.0f;
        var y = (float)(state[1] * ScaleY) - 100.0f;
        var v = state[2];

        // Draw velocity vector
        var velocityScale = 10.0f;
        renderer.DrawLine(
            new Vector2(x, y),
            new Vector2(x + (float)v * velocityScale, y),
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
        renderer.DrawText($"FRAME: {frameIndex + 1}/{states.Length}", 280, TopY, 2, Colors.Amber400);
        renderer.DrawText($"POSITION: ({state[0]:F2}, {state[1]:F2}) m", 280, TopY + 20, 2, Colors.Purple400);
        renderer.DrawText($"VELOCITY: {state[2]:F2} m/s", 280, TopY + 40, 2, Colors.Rose400);
        renderer.DrawText($"ANGLE: {control[0] * 180 / Math.PI:F1} deg", 280, TopY + 60, 2, Colors.Cyan400);

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
