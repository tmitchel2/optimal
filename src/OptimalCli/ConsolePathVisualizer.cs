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
/// Renders a real-time pseudo-graphical display of a 2D path in the console.
/// </summary>
internal static class ConsolePathVisualizer
{
    private const int Width = 80;
    private const int Height = 25;
    private static readonly object _lock = new object();

    /// <summary>
    /// Renders a path in the console as ASCII art.
    /// </summary>
    /// <param name="xData">X coordinates of the path.</param>
    /// <param name="yData">Y coordinates of the path.</param>
    /// <param name="title">Title to display above the visualization.</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    public static void RenderPath(double[] xData, double[] yData, string title, int iteration, double cost)
    {
        if (xData.Length == 0 || yData.Length == 0)
        {
            return;
        }

        lock (_lock)
        {
            // Clear previous display (move cursor to top-left and clear screen)
            Console.SetCursorPosition(0, 0);

            var sb = new StringBuilder();

            // Header
            sb.AppendLine($"╔{'═'.ToString().PadRight(Width - 2, '═')}╗");
            sb.AppendLine($"║ {title.PadRight(Width - 4)}║");
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

            // Add padding to bounds
            var xRange = xMax - xMin;
            var yRange = yMax - yMin;
            if (xRange < 1e-6) xRange = 1.0;
            if (yRange < 1e-6) yRange = 1.0;

            xMin -= xRange * 0.1;
            xMax += xRange * 0.1;
            yMin -= yRange * 0.1;
            yMax += yRange * 0.1;

            // Create character grid
            var grid = new char[Height, Width];
            for (var row = 0; row < Height; row++)
            {
                for (var col = 0; col < Width; col++)
                {
                    grid[row, col] = ' ';
                }
            }

            // Draw axes
            var zeroRow = Height - 1 - (int)((0 - yMin) / (yMax - yMin) * (Height - 1));
            var zeroCol = (int)((0 - xMin) / (xMax - xMin) * (Width - 1));

            if (zeroRow >= 0 && zeroRow < Height)
            {
                for (var col = 0; col < Width; col++)
                {
                    grid[zeroRow, col] = '─';
                }
            }

            if (zeroCol >= 0 && zeroCol < Width)
            {
                for (var row = 0; row < Height; row++)
                {
                    grid[row, zeroCol] = '│';
                }
            }

            if (zeroRow >= 0 && zeroRow < Height && zeroCol >= 0 && zeroCol < Width)
            {
                grid[zeroRow, zeroCol] = '┼';
            }

            // Draw the path
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
                var row = Height - 1 - (int)((y - yMin) / (yMax - yMin) * (Height - 1));

                if (row >= 0 && row < Height && col >= 0 && col < Width)
                {
                    if (i == 0)
                    {
                        grid[row, col] = 'S'; // Start
                    }
                    else if (i == xData.Length - 1)
                    {
                        grid[row, col] = 'E'; // End
                    }
                    else
                    {
                        // Use different characters based on path density
                        grid[row, col] = '●';
                    }
                }

                // Draw line segments between points
                if (i > 0)
                {
                    var prevX = xData[i - 1];
                    var prevY = yData[i - 1];

                    if (!double.IsNaN(prevX) && !double.IsInfinity(prevX) && !double.IsNaN(prevY) && !double.IsInfinity(prevY))
                    {
                        var prevCol = (int)((prevX - xMin) / (xMax - xMin) * (Width - 1));
                        var prevRow = Height - 1 - (int)((prevY - yMin) / (yMax - yMin) * (Height - 1));

                        // Simple line drawing (Bresenham-like)
                        DrawLine(grid, prevRow, prevCol, row, col);
                    }
                }
            }

            // Render grid to string
            for (var row = 0; row < Height; row++)
            {
                for (var col = 0; col < Width; col++)
                {
                    sb.Append(grid[row, col]);
                }
                sb.AppendLine();
            }

            // Footer with axis labels
            sb.AppendLine($"X: [{xMin:F2}, {xMax:F2}]  Y: [{yMin:F2}, {yMax:F2}]");

            // Output the entire frame at once
            Console.Write(sb.ToString());
        }
    }

    /// <summary>
    /// Draws a line between two points on the character grid.
    /// </summary>
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
}
