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
/// Renders a real-time graphical display of a cart-pole system using Radiant.
/// </summary>
internal static class RadiantCartPoleVisualizer
{
    private const int WindowWidth = 800;
    private const int WindowHeight = 600;
    private const float TrackY = 50.0f;
    private const float TrackLength = 600.0f;
    private const float CartWidth = 60.0f;
    private const float CartHeight = 40.0f;
    private const float PoleLength = 150.0f;
    private const float PixelsPerMeter = 100.0f;

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
            app.Run("Cart-Pole Swing-Up Optimization", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
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

        var state = states[frameIndex];
        var control = controls[frameIndex];

        var cartPos = state[0];
        var poleAngle = state[2];  // 0 = upright, π = hanging down
        var force = control[0];

        // Draw track
        DrawTrack(renderer);

        // Draw cart and pole
        DrawCartPole(renderer, cartPos, poleAngle, force);

        // Draw information text
        DrawInformation(renderer, cartPos, poleAngle, force, iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length);
    }

    private static void DrawTrack(Radiant.Graphics2D.Renderer2D renderer)
    {
        // Draw horizontal track
        const float TrackThickness = 4.0f;
        renderer.DrawRectangleFilled(-TrackLength / 2, TrackY - TrackThickness / 2, TrackLength, TrackThickness, Colors.Gray600);

        // Draw position markers at -2m, -1m, 0m, 1m, 2m
        for (var pos = -2; pos <= 2; pos++)
        {
            var x = pos * PixelsPerMeter;
            renderer.DrawLine(
                new Vector2(x, TrackY - 10),
                new Vector2(x, TrackY + 10),
                new Vector4(0.5f, 0.5f, 0.5f, 1.0f));

            // Label
            renderer.DrawText($"{pos}M", x - 15, TrackY + 20, 2, Colors.Gray400);
        }
    }

    private static void DrawCartPole(Radiant.Graphics2D.Renderer2D renderer, double cartPos, double poleAngle, double force)
    {
        // Cart position on screen
        var cartX = (float)(cartPos * PixelsPerMeter);
        var cartY = TrackY;

        // Draw cart
        renderer.DrawRectangleFilled(
            cartX - CartWidth / 2,
            cartY - CartHeight / 2,
            CartWidth,
            CartHeight,
            Colors.Sky600);

        // Draw cart outline
        renderer.DrawRectangleOutline(
            cartX - CartWidth / 2,
            cartY - CartHeight / 2,
            CartWidth,
            CartHeight,
            Colors.Sky400);

        // Draw wheels
        const float WheelRadius = 8.0f;
        renderer.DrawCircleFilled(cartX - 15, cartY + CartHeight / 2, WheelRadius, Colors.Slate800, 16);
        renderer.DrawCircleFilled(cartX + 15, cartY + CartHeight / 2, WheelRadius, Colors.Slate800, 16);

        // Calculate pole end position
        // poleAngle: 0 = upright (pointing up), π = down
        // In Radiant coordinates, Y increases upward
        var poleEndX = cartX + (float)(PoleLength * Math.Sin(poleAngle));
        var poleEndY = cartY - (float)(PoleLength * Math.Cos(poleAngle));

        // Draw pole
        renderer.DrawLine(
            new Vector2(cartX, cartY),
            new Vector2(poleEndX, poleEndY),
            new Vector4(0.9f, 0.9f, 0.9f, 1.0f));

        // Draw pole bob
        renderer.DrawCircleFilled(poleEndX, poleEndY, 15, Colors.Rose500, 24);
        renderer.DrawCircleOutline(poleEndX, poleEndY, 15, Colors.Rose400, 24);

        // Draw force indicator (arrow)
        if (Math.Abs(force) > 0.1)
        {
            var forceScale = 0.5f;
            var arrowLength = (float)(force * forceScale);
            var arrowY = cartY + 40;

            // Arrow shaft
            renderer.DrawLine(
                new Vector2(cartX, arrowY),
                new Vector2(cartX + arrowLength, arrowY),
                force > 0 ? Colors.Emerald500 : Colors.Red500);

            // Arrow head
            var headSize = 8.0f;
            var headDir = force > 0 ? 1 : -1;
            renderer.DrawLine(
                new Vector2(cartX + arrowLength, arrowY),
                new Vector2(cartX + arrowLength - (headDir * headSize), arrowY - headSize),
                force > 0 ? Colors.Emerald500 : Colors.Red500);
            renderer.DrawLine(
                new Vector2(cartX + arrowLength, arrowY),
                new Vector2(cartX + arrowLength - (headDir * headSize), arrowY + headSize),
                force > 0 ? Colors.Emerald500 : Colors.Red500);
        }
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer, double cartPos, double poleAngle, double force, int iteration, double cost, double maxViolation, double constraintTolerance, int frameIndex, int totalFrames)
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

        // Cart position
        renderer.DrawText($"CART POS: {cartPos:F2} M", 200, TopY + 20, 2, Colors.Purple400);

        // Pole angle in degrees
        var angleDegrees = poleAngle * 180.0 / Math.PI;
        renderer.DrawText($"POLE ANGLE: {angleDegrees:F1} DEG", 200, TopY + 40, 2, Colors.Cyan400);

        // Force
        renderer.DrawText($"FORCE: {force:F2} N", 200, TopY + 60, 2, Colors.Rose400);

        // Status text
        var statusText = Math.Abs(angleDegrees) < 10 ? "BALANCED!" : "BALANCING...";
        var statusColor = Math.Abs(angleDegrees) < 10 ? Colors.Emerald500 : Colors.Amber500;
        renderer.DrawText(statusText, -50, -200, 3, statusColor);
    }
}
