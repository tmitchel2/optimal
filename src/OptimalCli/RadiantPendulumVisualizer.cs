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
/// Renders a real-time graphical display of a pendulum using Radiant.
/// </summary>
internal static class RadiantPendulumVisualizer
{
    private const int WindowWidth = 800;
    private const int WindowHeight = 600;
    private const float PendulumLength = 200.0f; // Length in pixels
    private const float PivotX = 0.0f;
    private const float PivotY = -100.0f;

    private static double[][]? s_currentStates;
    private static double[][]? s_currentControls;
    private static int s_currentIteration;
    private static double s_currentCost;
    private static int s_currentFrameIndex;
    private static DateTime s_animationStartTime;
    private static readonly object s_lock = new();

    // Buffered next trajectory (waiting to be displayed)
    private static double[][]? s_nextStates;
    private static double[][]? s_nextControls;
    private static int s_nextIteration;
    private static double s_nextCost;
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
    public static void UpdateTrajectory(double[][] states, double[][] controls, int iteration, double cost)
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
            s_hasNextTrajectory = true;

            // Debug output
            Console.WriteLine($"[VIZ] Buffered Iter {iteration}: {states.Length} frames, cost={cost:F4}");
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
            app.Run("Pendulum Swing-Up Optimization", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
        }
        finally
        {
            // Signal cancellation when window closes
            if (s_cancellationTokenSource != null && !s_cancellationTokenSource.IsCancellationRequested)
            {
                Console.WriteLine("[VIZ] Window closed - requesting optimization cancellation");
                s_cancellationTokenSource.Cancel();
            }

            s_cancellationTokenSource?.Dispose();
            s_cancellationTokenSource = null;
        }
    }

    /// <summary>
    /// Renders an animated trajectory of the pendulum system (blocking call).
    /// </summary>
    /// <param name="states">Array of states over time [time_index][state_vars].</param>
    /// <param name="controls">Array of controls over time [time_index][control_vars].</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    public static void RenderTrajectory(double[][] states, double[][] controls, int iteration, double cost)
    {
        if (states.Length == 0 || controls.Length == 0)
        {
            return;
        }

        lock (s_lock)
        {
            s_currentStates = states;
            s_currentControls = controls;
            s_currentIteration = iteration;
            s_currentCost = cost;
            s_currentFrameIndex = 0;
            s_animationStartTime = DateTime.Now;
        }

        using var app = new RadiantApplication();
        app.Run("Pendulum Swing-Up Optimization", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
    }

    private static void RenderFrame(Radiant.Graphics2D.Renderer2D renderer)
    {
        double[][] states;
        double[][] controls;
        int iteration;
        double cost;
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
            // Slower animation: show each frame for ~100ms for better visibility
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
            frameIndex = s_currentFrameIndex;
        }

        var state = states[frameIndex];
        var control = controls[frameIndex];

        var theta = state[0];
        var torque = control[0];

        // Draw reference axes
        DrawReferenceAxes(renderer);

        // Draw pendulum
        DrawPendulum(renderer, theta);

        // Draw information text
        DrawInformation(renderer, theta, torque, iteration, cost, frameIndex, states.Length);
    }

    private static void DrawReferenceAxes(Radiant.Graphics2D.Renderer2D renderer)
    {
        // Draw horizontal line through pivot
        renderer.DrawLine(
            new Vector2(PivotX - 150, PivotY),
            new Vector2(PivotX + 150, PivotY),
            new Vector4(0.3f, 0.3f, 0.3f, 1.0f));

        // Draw vertical reference line (downward direction, θ=0)
        // In math coords, Y increases upward, so -Y is down
        renderer.DrawLine(
            new Vector2(PivotX, PivotY),
            new Vector2(PivotX, PivotY - 50),
            new Vector4(0.3f, 0.3f, 0.3f, 1.0f));

        // Draw upward reference line (θ=π)
        renderer.DrawLine(
            new Vector2(PivotX, PivotY),
            new Vector2(PivotX, PivotY + 50),
            new Vector4(0.3f, 0.5f, 0.3f, 1.0f));
    }

    private static void DrawPendulum(Radiant.Graphics2D.Renderer2D renderer, double theta)
    {
        // Calculate bob position
        // θ=0 should point down, θ=π should point up
        // Radiant uses math coordinates (Y increases upward), so we negate the Y offset
        var bobX = PivotX + (float)(PendulumLength * Math.Sin(theta));
        var bobY = PivotY - (float)(PendulumLength * Math.Cos(theta));

        // Draw pivot point
        renderer.DrawCircleFilled(PivotX, PivotY, 8, Colors.Slate400, 16);

        // Draw rod
        renderer.DrawLine(
            new Vector2(PivotX, PivotY),
            new Vector2(bobX, bobY),
            new Vector4(0.9f, 0.9f, 0.9f, 1.0f));

        // Draw bob
        renderer.DrawCircleFilled(bobX, bobY, 20, Colors.Sky500, 32);

        // Draw bob outline
        renderer.DrawCircleOutline(bobX, bobY, 20, Colors.Sky300, 32);

        // Draw velocity indicator (angular velocity)
        // Not drawn in this simple version, but could show as an arrow
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer, double theta, double torque, int iteration, double cost, int frameIndex, int totalFrames)
    {
        // Draw text information at the top
        const float TopY = -270.0f;

        // Iteration and cost
        renderer.DrawText($"ITERATION: {iteration}", -380, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"COST: {cost:F4}", -380, TopY + 20, 2, Colors.Sky400);

        // Frame info
        renderer.DrawText($"FRAME: {frameIndex + 1}/{totalFrames}", 200, TopY, 2, Colors.Amber400);

        // Angle in degrees
        var angleDegrees = theta * 180.0 / Math.PI;
        renderer.DrawText($"ANGLE: {angleDegrees:F1} DEG", 200, TopY + 20, 2, Colors.Purple400);

        // Torque
        renderer.DrawText($"TORQUE: {torque:F2} NM", 200, TopY + 40, 2, Colors.Rose400);

        // Draw a progress bar at the bottom showing angle
        var normalizedAngle = (angleDegrees + 180.0) / 360.0; // Map -180..180 to 0..1

        const float BarWidth = 600.0f;
        const float BarHeight = 20.0f;
        const float BarX = -BarWidth / 2;
        const float BarY = 250.0f;

        // Background bar
        renderer.DrawRectangleFilled(BarX, BarY, BarWidth, BarHeight, Colors.Slate700);

        // Progress indicator
        var progressX = BarX + (float)(normalizedAngle * BarWidth);
        renderer.DrawCircleFilled(progressX, BarY + (BarHeight / 2), 15, Colors.Emerald500, 16);

        // Draw markers at 0° and 180°
        const float Marker0 = BarX + (float)(180.0 / 360.0 * BarWidth);
        const float Marker180 = BarX + (float)(360.0 / 360.0 * BarWidth);
        renderer.DrawLine(
            new Vector2(Marker0, BarY - 5),
            new Vector2(Marker0, BarY + BarHeight + 5),
            new Vector4(0.8f, 0.3f, 0.3f, 1.0f));
        renderer.DrawLine(
            new Vector2(Marker180, BarY - 5),
            new Vector2(Marker180, BarY + BarHeight + 5),
            new Vector4(0.3f, 0.8f, 0.3f, 1.0f));

        // Frame counter bar
        if (totalFrames > 0)
        {
            const float FrameBarHeight = 8.0f;
            const float FrameBarY = 270.0f;
            renderer.DrawRectangleFilled(BarX, FrameBarY, BarWidth, FrameBarHeight, Colors.Slate700);
            var frameProgress = (float)frameIndex / totalFrames * BarWidth;
            renderer.DrawRectangleFilled(BarX, FrameBarY, frameProgress, FrameBarHeight, Colors.Sky600);
        }

        // Draw torque indicator (as a colored circle)
        var torqueIndicatorRadius = (float)Math.Abs(torque) * 5.0f;
        var torqueColor = torque > 0
            ? new Vector4(0.3f, 0.8f, 0.3f, 0.8f) // Green for positive
            : new Vector4(0.8f, 0.3f, 0.3f, 0.8f); // Red for negative

        if (torqueIndicatorRadius > 1.0f)
        {
            renderer.DrawCircleOutline(PivotX, PivotY, torqueIndicatorRadius + 15, torqueColor, 32);
        }
    }
}
