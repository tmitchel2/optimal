/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Text;

namespace OptimalCli;

/// <summary>
/// Renders a real-time pseudo-graphical display of a pendulum in the console.
/// </summary>
internal static class PendulumVisualizer
{
    private const int Width = 80;
    private const int Height = 25;
    private const int AnimationHeight = 17;  // Rows 4-20
    private const int PendulumLength = 8;    // Character length of pendulum rod
    private static readonly object _lock = new object();

    /// <summary>
    /// Renders a pendulum in the console as ASCII art.
    /// </summary>
    /// <param name="theta">Angular position in radians (0=down, π=up).</param>
    /// <param name="thetaDot">Angular velocity in rad/s.</param>
    /// <param name="torque">Control torque in N·m.</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    public static void RenderPendulum(double theta, double thetaDot, double torque, int iteration, double cost)
    {
        lock (_lock)
        {
            Console.SetCursorPosition(0, 0);

            var sb = new StringBuilder(2000);  // Pre-allocate capacity

            // Header (rows 0-3)
            AppendHeader(sb, iteration, cost);

            // Create animation grid (17 rows × 80 columns)
            var grid = new char[AnimationHeight, Width];
            InitializeGrid(grid);
            DrawReferenceLines(grid);
            DrawPendulum(grid, theta);

            // Append grid to output
            AppendGrid(sb, grid);

            // Footer (rows 21-24)
            AppendFooter(sb, theta, thetaDot, torque);

            // Output the entire frame at once
            Console.Write(sb.ToString());
        }
    }

    /// <summary>
    /// Clears the console for a fresh visualization.
    /// </summary>
    public static void Initialize()
    {
        Console.Clear();
        Console.CursorVisible = false;
    }

    /// <summary>
    /// Restores console state after visualization.
    /// </summary>
    public static void Cleanup()
    {
        Console.CursorVisible = true;
    }

    private static void AppendHeader(StringBuilder sb, int iteration, double cost)
    {
        sb.AppendLine($"╔{'═'.ToString().PadRight(Width - 2, '═')}╗");
        sb.AppendLine($"║ {"PENDULUM SWING-UP OPTIMIZATION".PadRight(Width - 4)}║");
        sb.AppendLine($"║ Iteration: {iteration,-10} Cost: {cost:F6}{string.Empty.PadRight(Width - 40)}║");
        sb.AppendLine($"╚{'═'.ToString().PadRight(Width - 2, '═')}╝");
    }

    private static void AppendFooter(StringBuilder sb, double theta, double thetaDot, double torque)
    {
        var angleDegrees = theta * 180.0 / Math.PI;

        sb.AppendLine();
        sb.AppendLine($"┌{"─".PadRight(Width - 2, '─')}┐");
        sb.Append($"│ Angle: {angleDegrees,7:F1}°  │  Velocity: {thetaDot,7:F2} rad/s  │  Torque: {torque,7:F2} N·m");
        sb.Append(string.Empty.PadRight(Width - 79));
        sb.AppendLine("│");
        sb.AppendLine($"└{"─".PadRight(Width - 2, '─')}┘");
    }

    private static void InitializeGrid(char[,] grid)
    {
        for (var row = 0; row < AnimationHeight; row++)
        {
            for (var col = 0; col < Width; col++)
            {
                grid[row, col] = ' ';
            }
        }
    }

    private static void DrawReferenceLines(char[,] grid)
    {
        var centerRow = 8;  // Middle of 17-row grid
        var centerCol = 40; // Middle of 80-column width

        // Draw horizontal reference line through pivot
        for (var col = centerCol - 10; col <= centerCol + 10; col++)
        {
            if (InBounds(grid, centerRow, col))
            {
                grid[centerRow, col] = '─';
            }
        }

        // Draw vertical reference line (downward direction, θ=0)
        for (var row = centerRow + 1; row < AnimationHeight && row < centerRow + 4; row++)
        {
            if (InBounds(grid, row, centerCol))
            {
                grid[row, centerCol] = '│';
            }
        }
    }

    private static void DrawPendulum(char[,] grid, double theta)
    {
        var centerRow = 8;  // Middle of 17-row grid
        var centerCol = 40; // Middle of 80-column width

        // Normalize angle to [-π, π]
        var angle = NormalizeAngle(theta);

        // Compute bob position
        // Note: In screen coordinates, rows increase downward
        // θ=0 should point down (positive row offset), θ=π should point up (negative row offset)
        var bobCol = centerCol + (int)Math.Round(PendulumLength * Math.Sin(angle));
        var bobRow = centerRow + (int)Math.Round(PendulumLength * Math.Cos(angle));

        // Draw pivot
        if (InBounds(grid, centerRow, centerCol))
        {
            grid[centerRow, centerCol] = '┼';
        }

        // Draw rod using Bresenham line algorithm
        DrawLine(grid, centerRow, centerCol, bobRow, bobCol, angle);

        // Draw bob (overwrites line endpoint)
        if (InBounds(grid, bobRow, bobCol))
        {
            grid[bobRow, bobCol] = '●';
        }
    }

    private static void DrawLine(char[,] grid, int row0, int col0, int row1, int col1, double theta)
    {
        var lineChar = GetLineCharacter(theta);

        var dx = Math.Abs(col1 - col0);
        var dy = Math.Abs(row1 - row0);
        var sx = col0 < col1 ? 1 : -1;
        var sy = row0 < row1 ? 1 : -1;
        var err = dx - dy;

        var currentRow = row0;
        var currentCol = col0;

        while (true)
        {
            if (InBounds(grid, currentRow, currentCol))
            {
                // Don't overwrite pivot or reference axes
                if (grid[currentRow, currentCol] == ' ')
                {
                    grid[currentRow, currentCol] = lineChar;
                }
            }

            if (currentRow == row1 && currentCol == col1)
            {
                break;
            }

            var e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                currentCol += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                currentRow += sy;
            }
        }
    }

    private static char GetLineCharacter(double theta)
    {
        // Normalize angle to [-π, π]
        var angle = NormalizeAngle(theta);

        // Convert to degrees for easier angle range checking
        var degrees = Math.Abs(angle * 180.0 / Math.PI);

        // Map angle to character based on orientation
        if (degrees < 22.5 || degrees > 157.5)
        {
            return '│';  // Nearly vertical
        }
        else if (degrees > 67.5 && degrees < 112.5)
        {
            return '─';  // Nearly horizontal
        }
        else if ((angle > 0 && angle < Math.PI / 2) || (angle < -Math.PI / 2))
        {
            return '/';  // Positive slope in screen coords
        }
        else
        {
            return '\\'; // Negative slope in screen coords
        }
    }

    private static double NormalizeAngle(double theta)
    {
        var angle = theta % (2.0 * Math.PI);
        if (angle > Math.PI)
        {
            angle -= 2.0 * Math.PI;
        }
        if (angle < -Math.PI)
        {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    private static void AppendGrid(StringBuilder sb, char[,] grid)
    {
        for (var row = 0; row < AnimationHeight; row++)
        {
            for (var col = 0; col < Width; col++)
            {
                sb.Append(grid[row, col]);
            }
            sb.AppendLine();
        }
    }

    private static bool InBounds(char[,] grid, int row, int col)
    {
        return row >= 0 && row < AnimationHeight && col >= 0 && col < Width;
    }
}
