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
/// Renders a real-time pseudo-graphical display of a Dubins car path in the console.
/// </summary>
internal static class DubinsCarVisualizer
{
    private const int Width = 80;
    private const int Height = 25;
    private const int AnimationHeight = 18;  // Rows for the path grid (total 25 = 4 header + 18 grid + 3 footer)
    private static readonly object _lock = new object();

    /// <summary>
    /// Renders a Dubins car path in the console as ASCII art.
    /// </summary>
    /// <param name="xData">X coordinates of the path.</param>
    /// <param name="yData">Y coordinates of the path.</param>
    /// <param name="headings">Heading angles (θ) in radians.</param>
    /// <param name="turnRate">Current turning rate (ω) in rad/s.</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    public static void RenderPath(double[] xData, double[] yData, double[] headings, double turnRate, int iteration, double cost)
    {
        if (xData.Length == 0 || yData.Length == 0)
        {
            return;
        }

        lock (_lock)
        {
            Console.SetCursorPosition(0, 0);

            var sb = new StringBuilder(2000);

            // Header
            sb.AppendLine($"╔{'═'.ToString().PadRight(Width - 2, '═')}╗");
            sb.AppendLine($"║ {"DUBINS CAR OPTIMIZATION - Minimum Path with Curvature Constraint".PadRight(Width - 4)}║");
            sb.AppendLine($"║ Iteration: {iteration,-10} Cost: {cost:F6}{string.Empty.PadRight(Width - 40)}║");
            sb.AppendLine($"╚{'═'.ToString().PadRight(Width - 2, '═')}╝");

            // Find data bounds
            var xMin = double.MaxValue;
            var xMax = double.MinValue;
            var yMin = double.MaxValue;
            var yMax = double.MinValue;

            for (var i = 0; i < xData.Length; i++)
            {
                if (!double.IsNaN(xData[i]) && !double.IsInfinity(xData[i]))
                {
                    xMin = Math.Min(xMin, xData[i]);
                    xMax = Math.Max(xMax, xData[i]);
                }
                if (!double.IsNaN(yData[i]) && !double.IsInfinity(yData[i]))
                {
                    yMin = Math.Min(yMin, yData[i]);
                    yMax = Math.Max(yMax, yData[i]);
                }
            }

            // Add padding to bounds and ensure equal scaling for both axes
            var xRange = xMax - xMin;
            var yRange = yMax - yMin;
            if (xRange < 1e-6) xRange = 1.0;
            if (yRange < 1e-6) yRange = 1.0;

            // Use the same scale factor for both axes to preserve aspect ratio
            var maxRange = Math.Max(xRange, yRange);
            var xCenter = (xMin + xMax) / 2.0;
            var yCenter = (yMin + yMax) / 2.0;

            // Add 10% padding
            var paddedRange = maxRange * 1.1;

            xMin = xCenter - paddedRange / 2.0;
            xMax = xCenter + paddedRange / 2.0;
            yMin = yCenter - paddedRange / 2.0;
            yMax = yCenter + paddedRange / 2.0;

            // Create character grid
            var grid = new char[AnimationHeight, Width];
            for (var row = 0; row < AnimationHeight; row++)
            {
                for (var col = 0; col < Width; col++)
                {
                    grid[row, col] = ' ';
                }
            }

            // Draw axes
            var zeroRow = AnimationHeight - 1 - (int)((0 - yMin) / (yMax - yMin) * (AnimationHeight - 1));
            var zeroCol = (int)((0 - xMin) / (xMax - xMin) * (Width - 1));

            if (zeroRow >= 0 && zeroRow < AnimationHeight)
            {
                for (var col = 0; col < Width; col++)
                {
                    grid[zeroRow, col] = '─';
                }
            }

            if (zeroCol >= 0 && zeroCol < Width)
            {
                for (var row = 0; row < AnimationHeight; row++)
                {
                    grid[row, zeroCol] = '│';
                }
            }

            if (zeroRow >= 0 && zeroRow < AnimationHeight && zeroCol >= 0 && zeroCol < Width)
            {
                grid[zeroRow, zeroCol] = '┼';
            }

            // Draw the path with line segments
            for (var i = 0; i < xData.Length; i++)
            {
                var x = xData[i];
                var y = yData[i];

                if (double.IsNaN(x) || double.IsInfinity(x) || double.IsNaN(y) || double.IsInfinity(y))
                {
                    continue;
                }

                // Map to grid coordinates (flip y because console row 0 is at top)
                var col = (int)((x - xMin) / (xMax - xMin) * (Width - 1));
                var row = AnimationHeight - 1 - (int)((y - yMin) / (yMax - yMin) * (AnimationHeight - 1));

                // Draw line segments between points
                if (i > 0)
                {
                    var prevX = xData[i - 1];
                    var prevY = yData[i - 1];

                    if (!double.IsNaN(prevX) && !double.IsInfinity(prevX) && !double.IsNaN(prevY) && !double.IsInfinity(prevY))
                    {
                        var prevCol = (int)((prevX - xMin) / (xMax - xMin) * (Width - 1));
                        var prevRow = AnimationHeight - 1 - (int)((prevY - yMin) / (yMax - yMin) * (AnimationHeight - 1));

                        DrawLine(grid, prevRow, prevCol, row, col);
                    }
                }

                // Draw position markers
                if (row >= 0 && row < AnimationHeight && col >= 0 && col < Width)
                {
                    if (i == 0)
                    {
                        grid[row, col] = 'S'; // Start
                    }
                    else if (i == xData.Length - 1)
                    {
                        // Draw car at final position with heading direction
                        var heading = headings[i];
                        grid[row, col] = GetCarCharacter(heading);
                    }
                    else if (grid[row, col] == ' ' || grid[row, col] == '·')
                    {
                        grid[row, col] = '·'; // Path point
                    }
                }
            }

            // Render grid to string
            for (var row = 0; row < AnimationHeight; row++)
            {
                for (var col = 0; col < Width; col++)
                {
                    sb.Append(grid[row, col]);
                }
                sb.AppendLine();
            }

            // Footer with metrics
            var finalX = xData[^1];
            var finalY = yData[^1];
            var finalHeading = headings[^1] * 180.0 / Math.PI;

            sb.AppendLine($"┌{"─".PadRight(Width - 2, '─')}┐");
            var footerText = $"│ Pos: ({finalX,5:F2},{finalY,5:F2}) │ Head: {finalHeading,6:F1}° │ Turn: {turnRate,5:F2} rad/s";
            sb.Append(footerText);
            sb.Append(string.Empty.PadRight(Width - footerText.Length - 1));
            sb.AppendLine("│");
            sb.AppendLine($"└{"─".PadRight(Width - 2, '─')}┘");

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

    private static void DrawLine(char[,] grid, int row0, int col0, int row1, int col1)
    {
        var height = grid.GetLength(0);
        var width = grid.GetLength(1);

        var dx = Math.Abs(col1 - col0);
        var dy = Math.Abs(row1 - row0);
        var sx = col0 < col1 ? 1 : -1;
        var sy = row0 < row1 ? 1 : -1;
        var err = dx - dy;

        var currentRow = row0;
        var currentCol = col0;

        while (true)
        {
            if (currentRow >= 0 && currentRow < height && currentCol >= 0 && currentCol < width)
            {
                if (grid[currentRow, currentCol] == ' ' || grid[currentRow, currentCol] == '─' || grid[currentRow, currentCol] == '│')
                {
                    grid[currentRow, currentCol] = '·';
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

    private static char GetCarCharacter(double heading)
    {
        // Normalize heading to [0, 2π)
        var angle = heading % (2.0 * Math.PI);
        if (angle < 0)
        {
            angle += 2.0 * Math.PI;
        }

        // Convert to degrees for easier comparison
        var degrees = angle * 180.0 / Math.PI;

        // Map heading to arrow character
        // 0° = East (right), 90° = North (up), 180° = West (left), 270° = South (down)
        if (degrees >= 337.5 || degrees < 22.5)
        {
            return '→'; // East
        }
        else if (degrees >= 22.5 && degrees < 67.5)
        {
            return '↗'; // Northeast
        }
        else if (degrees >= 67.5 && degrees < 112.5)
        {
            return '↑'; // North
        }
        else if (degrees >= 112.5 && degrees < 157.5)
        {
            return '↖'; // Northwest
        }
        else if (degrees >= 157.5 && degrees < 202.5)
        {
            return '←'; // West
        }
        else if (degrees >= 202.5 && degrees < 247.5)
        {
            return '↙'; // Southwest
        }
        else if (degrees >= 247.5 && degrees < 292.5)
        {
            return '↓'; // South
        }
        else // degrees >= 292.5 && degrees < 337.5
        {
            return '↘'; // Southeast
        }
    }
}
