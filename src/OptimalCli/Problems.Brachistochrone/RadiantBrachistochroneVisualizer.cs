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
    private const int WindowWidth = 1200;
    private const int WindowHeight = 800;
    private const float PixelsPerMeter = 150.0f;  // Scale for visualization
    private const float BallRadius = 8.0f;

    private static double[][]? s_currentStates;
    private static double[][]? s_currentControls;
    private static int s_currentIteration;
    private static double s_currentCost;
    private static double s_currentMaxViolation;
    private static double s_currentConstraintTolerance;
    private static double s_xFinal;
    private static double s_yFinal;
    private static double s_alpha;
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
    private static double s_nextXFinal;
    private static double s_nextYFinal;
    private static double s_nextAlpha;
    private static bool s_hasNextTrajectory;

    // Cancellation token
    private static CancellationTokenSource? s_cancellationTokenSource;

    public static CancellationToken CancellationToken => s_cancellationTokenSource?.Token ?? CancellationToken.None;

    public static void UpdateTrajectory(double[][] states, double[][] controls, int iteration, double cost, double maxViolation, double constraintTolerance, double xFinal, double yFinal, double alpha)
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
            s_nextXFinal = xFinal;
            s_nextYFinal = yFinal;
            s_nextAlpha = alpha;
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
            app.Run("Brachistochrone Problem", WindowWidth, WindowHeight, RenderFrame, Colors.Slate900);
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
        double xFinal;
        double yFinal;
        double alpha;
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
                    s_xFinal = s_nextXFinal;
                    s_yFinal = s_nextYFinal;
                    s_alpha = s_nextAlpha;
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
            const double FrameDuration = 50.0;
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
                s_xFinal = s_nextXFinal;
                s_yFinal = s_nextYFinal;
                s_alpha = s_nextAlpha;
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
            xFinal = s_xFinal;
            yFinal = s_yFinal;
            alpha = s_alpha;
            frameIndex = s_currentFrameIndex;
        }

        var state = states[frameIndex];
        var s = state[0];
        var d = state[1];
        var v = state[2];

        // Convert (s, d) coordinates to (x, y) for visualization
        var (x, y) = ConvertToXY(s, d, alpha);

        // Draw reference elements
        DrawStartAndEndPoints(renderer, xFinal, yFinal);
        DrawDiagonalLine(renderer, xFinal, yFinal);
        DrawTrajectory(renderer, states, alpha);
        DrawBall(renderer, x, y, v);
        DrawGraphs(renderer, states, controls, frameIndex, xFinal, yFinal, alpha);
        DrawInformation(renderer, x, y, v, s, d, iteration, cost, maxViolation, constraintTolerance, frameIndex, states.Length);
    }

    private static (double x, double y) ConvertToXY(double s, double d, double alpha)
    {
        // Convert from (s, d) to (x, y)
        // s is along the diagonal, d is perpendicular to it
        var cosAlpha = Math.Cos(alpha);
        var sinAlpha = Math.Sin(alpha);
        
        // Position along diagonal
        var xDiag = s * cosAlpha;
        var yDiag = s * sinAlpha;
        
        // Perpendicular offset (perpendicular vector is (-sinAlpha, cosAlpha))
        var x = xDiag - d * sinAlpha;
        var y = yDiag + d * cosAlpha;
        
        return (x, y);
    }

    private static void DrawStartAndEndPoints(Radiant.Graphics2D.Renderer2D renderer, double xFinal, double yFinal)
    {
        // Draw start point (origin)
        var startX = -200.0f;
        var startY = 150.0f;
        renderer.DrawCircleFilled(startX, startY, 10, Colors.Emerald500, 16);
        renderer.DrawText("START", startX - 30, startY - 25, 1.5f, Colors.Emerald400);

        // Draw end point
        var endX = startX + (float)(xFinal * PixelsPerMeter);
        var endY = startY + (float)(yFinal * PixelsPerMeter);
        renderer.DrawCircleFilled(endX, endY, 10, Colors.Rose500, 16);
        renderer.DrawText("END", endX - 20, endY + 15, 1.5f, Colors.Rose400);
    }

    private static void DrawDiagonalLine(Radiant.Graphics2D.Renderer2D renderer, double xFinal, double yFinal)
    {
        var startX = -200.0f;
        var startY = 150.0f;
        var endX = startX + (float)(xFinal * PixelsPerMeter);
        var endY = startY + (float)(yFinal * PixelsPerMeter);

        renderer.DrawLine(
            new Vector2(startX, startY),
            new Vector2(endX, endY),
            new Vector4(0.5f, 0.5f, 0.5f, 0.5f));
    }

    private static void DrawTrajectory(Radiant.Graphics2D.Renderer2D renderer, double[][] states, double alpha)
    {
        var startX = -200.0f;
        var startY = 150.0f;

        for (var i = 1; i < states.Length; i++)
        {
            var s1 = states[i - 1][0];
            var d1 = states[i - 1][1];
            var s2 = states[i][0];
            var d2 = states[i][1];

            var (x1, y1) = ConvertToXY(s1, d1, alpha);
            var (x2, y2) = ConvertToXY(s2, d2, alpha);

            var px1 = startX + (float)(x1 * PixelsPerMeter);
            var py1 = startY + (float)(y1 * PixelsPerMeter);
            var px2 = startX + (float)(x2 * PixelsPerMeter);
            var py2 = startY + (float)(y2 * PixelsPerMeter);

            var alpha_val = 0.3f + 0.5f * (float)i / states.Length;
            var color = new Vector4(0.4f, 0.7f, 1.0f, alpha_val);

            renderer.DrawLine(new Vector2(px1, py1), new Vector2(px2, py2), color);
        }
    }

    private static void DrawBall(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double v)
    {
        var startX = -200.0f;
        var startY = 150.0f;
        var ballX = startX + (float)(x * PixelsPerMeter);
        var ballY = startY + (float)(y * PixelsPerMeter);

        // Draw ball with velocity-based color
        var speedFactor = Math.Min(1.0f, (float)(v / 10.0));
        var ballColor = new Vector4(1.0f - speedFactor * 0.5f, 0.3f + speedFactor * 0.5f, 0.2f, 1.0f);
        
        renderer.DrawCircleFilled(ballX, ballY, BallRadius, ballColor, 16);
        renderer.DrawCircleOutline(ballX, ballY, BallRadius, Colors.White, 16);

        // Draw velocity vector
        if (v > 0.1)
        {
            var velLength = (float)(v * 10.0);
            renderer.DrawLine(
                new Vector2(ballX, ballY),
                new Vector2(ballX, ballY + velLength),
                Colors.Yellow400);
        }
    }

    private static void DrawInformation(Radiant.Graphics2D.Renderer2D renderer, double x, double y, double v, double s, double d, int iteration, double cost, double maxViolation, double constraintTolerance, int frameIndex, int totalFrames)
    {
        const float TopY = -370.0f;
        const float LeftX = -580.0f;

        // Optimization info
        renderer.DrawText($"ITERATION: {iteration}", LeftX, TopY, 2, Colors.Emerald400);
        renderer.DrawText($"TIME: {cost:F4} S", LeftX, TopY + 20, 2, Colors.Sky400);

        var convergenceRatio = constraintTolerance > 0 ? maxViolation / constraintTolerance : 0.0;
        var isConverged = maxViolation < constraintTolerance;
        var convergenceColor = isConverged ? Colors.Emerald500 : (convergenceRatio < 10.0 ? Colors.Amber500 : Colors.Rose500);
        renderer.DrawText($"CONVERGENCE: {maxViolation:E2}/{constraintTolerance:E2}", LeftX, TopY + 40, 2, convergenceColor);

        // State info
        renderer.DrawText($"FRAME: {frameIndex + 1}/{totalFrames}", LeftX + 300, TopY, 2, Colors.Amber400);
        renderer.DrawText($"POSITION: ({x:F3}, {y:F3}) M", LeftX + 300, TopY + 20, 2, Colors.Purple400);
        renderer.DrawText($"VELOCITY: {v:F2} M/S", LeftX + 300, TopY + 40, 2, Colors.Cyan400);
        renderer.DrawText($"S (DIAGONAL): {s:F3} M", LeftX + 300, TopY + 60, 2, Colors.Lime400);
        renderer.DrawText($"D (PERPENDICULAR): {d:F3} M", LeftX + 300, TopY + 80, 2, Colors.Pink400);
    }

    private static void DrawGraphs(Radiant.Graphics2D.Renderer2D renderer, double[][] states, double[][] controls, int currentFrame, double xFinal, double yFinal, double alpha)
    {
        const float GraphPanelX = 250.0f;
        const float GraphPanelY = -250.0f;
        const float GraphWidth = 300.0f;
        const float GraphHeight = 70.0f;
        const float GraphSpacing = 85.0f;

        // Convert states to x,y coordinates for plotting
        var xyTrajectory = new double[states.Length][];
        for (var i = 0; i < states.Length; i++)
        {
            var (x, y) = ConvertToXY(states[i][0], states[i][1], alpha);
            xyTrajectory[i] = new[] { x, y, states[i][2] };  // [x, y, v]
        }

        DrawSingleGraph(renderer, xyTrajectory, 0, "X POSITION (M)", GraphPanelX, GraphPanelY, GraphWidth, GraphHeight, currentFrame, Colors.Purple400);
        DrawSingleGraph(renderer, xyTrajectory, 1, "Y POSITION (M)", GraphPanelX, GraphPanelY + GraphSpacing, GraphWidth, GraphHeight, currentFrame, Colors.Cyan400);
        DrawSingleGraph(renderer, xyTrajectory, 2, "VELOCITY (M/S)", GraphPanelX, GraphPanelY + 2 * GraphSpacing, GraphWidth, GraphHeight, currentFrame, Colors.Emerald400);
        DrawControlGraph(renderer, controls, "THETA (RAD)", GraphPanelX, GraphPanelY + 3 * GraphSpacing, GraphWidth, GraphHeight, currentFrame, Colors.Rose400);
    }

    private static void DrawSingleGraph(Radiant.Graphics2D.Renderer2D renderer, double[][] data, int stateIndex, string label, float x, float y, float width, float height, int currentFrame, Vector4 color)
    {
        if (data.Length < 2)
        {
            return;
        }

        renderer.DrawText(label, x - 10, y - 15, 1.5f, Colors.Gray400);
        renderer.DrawRectangleFilled(x, y, width, height, new Vector4(0.1f, 0.1f, 0.1f, 0.8f));
        renderer.DrawRectangleOutline(x, y, width, height, Colors.Gray600);

        var minVal = double.MaxValue;
        var maxVal = double.MinValue;
        for (var i = 0; i < data.Length; i++)
        {
            var val = data[i][stateIndex];
            if (val < minVal) minVal = val;
            if (val > maxVal) maxVal = val;
        }

        var range = maxVal - minVal;
        if (range < 1e-6) range = 1.0;
        minVal -= range * 0.1;
        maxVal += range * 0.1;
        range = maxVal - minVal;

        renderer.DrawText($"{maxVal:F2}", x - 50, y + height, 1.2f, Colors.Gray500);
        renderer.DrawText($"{minVal:F2}", x - 50, y, 1.2f, Colors.Gray500);

        for (var i = 1; i < data.Length; i++)
        {
            var val1 = data[i - 1][stateIndex];
            var val2 = data[i][stateIndex];

            var x1 = x + (float)(i - 1) / (data.Length - 1) * width;
            var x2 = x + (float)i / (data.Length - 1) * width;
            var y1 = y + (float)((val1 - minVal) / range) * height;
            var y2 = y + (float)((val2 - minVal) / range) * height;

            var lineColor = i <= currentFrame ? color : new Vector4(color.X * 0.3f, color.Y * 0.3f, color.Z * 0.3f, 0.5f);
            renderer.DrawLine(new Vector2(x1, y1), new Vector2(x2, y2), lineColor);
        }

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

        renderer.DrawText(label, x - 10, y - 15, 1.5f, Colors.Gray400);
        renderer.DrawRectangleFilled(x, y, width, height, new Vector4(0.1f, 0.1f, 0.1f, 0.8f));
        renderer.DrawRectangleOutline(x, y, width, height, Colors.Gray600);

        var minVal = double.MaxValue;
        var maxVal = double.MinValue;
        for (var i = 0; i < controls.Length; i++)
        {
            var val = controls[i][0];
            if (val < minVal) minVal = val;
            if (val > maxVal) maxVal = val;
        }

        minVal = Math.Min(minVal, 0.0);
        var range = maxVal - minVal;
        if (range < 1e-6) range = 1.0;
        minVal -= range * 0.1;
        maxVal += range * 0.1;
        range = maxVal - minVal;

        renderer.DrawText($"{maxVal:F2}", x - 50, y + height, 1.2f, Colors.Gray500);
        renderer.DrawText($"{minVal:F2}", x - 50, y, 1.2f, Colors.Gray500);

        if (minVal <= 0 && maxVal >= 0)
        {
            var zeroY = y + (float)((0 - minVal) / range) * height;
            renderer.DrawLine(new Vector2(x, zeroY), new Vector2(x + width, zeroY), new Vector4(0.5f, 0.5f, 0.5f, 0.5f));
        }

        for (var i = 1; i < controls.Length; i++)
        {
            var val1 = controls[i - 1][0];
            var val2 = controls[i][0];

            var x1 = x + (float)(i - 1) / (controls.Length - 1) * width;
            var x2 = x + (float)i / (controls.Length - 1) * width;
            var y1 = y + (float)((val1 - minVal) / range) * height;
            var y2 = y + (float)((val2 - minVal) / range) * height;

            var lineColor = i <= currentFrame ? color : new Vector4(color.X * 0.3f, color.Y * 0.3f, color.Z * 0.3f, 0.5f);
            renderer.DrawLine(new Vector2(x1, y1), new Vector2(x2, y2), lineColor);
        }

        if (currentFrame < controls.Length)
        {
            var currentVal = controls[currentFrame][0];
            var markerX = x + (float)currentFrame / (controls.Length - 1) * width;
            var markerY = y + (float)((currentVal - minVal) / range) * height;
            renderer.DrawCircleFilled(markerX, markerY, 3, color, 8);
        }
    }
}
