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
/// Renders a real-time pseudo-graphical display of a cart-pole system in the console.
/// </summary>
internal static class CartPoleVisualizer
{
    private const int Width = 80;
    private const int Height = 25;
    private const int AnimationHeight = 17;  // Rows 4-20
    private const int PoleLength = 7;        // Character length of pole
    private const int CartWidth = 6;         // Width of cart in characters
    private static readonly object _lock = new object();

    /// <summary>
    /// Renders an animated trajectory of the cart-pole system.
    /// Plays through the entire trajectory and loops until the display duration is reached.
    /// </summary>
    /// <param name="states">Array of states over time [time_index][state_vars].</param>
    /// <param name="controls">Array of controls over time [time_index][control_vars].</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    /// <param name="displayDurationMs">Total time to display/loop the animation in milliseconds.</param>
    public static void RenderTrajectory(double[][] states, double[][] controls, int iteration, double cost, int displayDurationMs = 1000)
    {
        if (states.Length == 0 || controls.Length == 0)
        {
            return;
        }

        var startTime = DateTime.Now;
        var frameDelayMs = Math.Max(10, displayDurationMs / (states.Length * 2)); // At least 2 loops

        while ((DateTime.Now - startTime).TotalMilliseconds < displayDurationMs)
        {
            for (var i = 0; i < states.Length; i++)
            {
                var state = states[i];
                var control = controls[i];

                var cartPos = state[0];
                var cartVel = state[1];
                var poleAngle = state[2];
                var poleVel = state[3];
                var force = control[0];

                RenderCartPole(cartPos, cartVel, poleAngle, poleVel, force, iteration, cost, i, states.Length);

                System.Threading.Thread.Sleep(frameDelayMs);

                // Check if we've exceeded display duration
                if ((DateTime.Now - startTime).TotalMilliseconds >= displayDurationMs)
                {
                    return;
                }
            }

            // Brief pause between loops to show animation is restarting
            System.Threading.Thread.Sleep(200);
        }
    }

    /// <summary>
    /// Renders a single frame of the cart-pole system in the console as ASCII art.
    /// </summary>
    /// <param name="cartPos">Cart position in meters.</param>
    /// <param name="cartVel">Cart velocity in m/s.</param>
    /// <param name="poleAngle">Pole angle in radians (0=down, π=up).</param>
    /// <param name="poleVel">Pole angular velocity in rad/s.</param>
    /// <param name="force">Control force in N.</param>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    /// <param name="frameIndex">Current frame index in trajectory (optional).</param>
    /// <param name="totalFrames">Total frames in trajectory (optional).</param>
    public static void RenderCartPole(double cartPos, double cartVel, double poleAngle, double poleVel, double force, int iteration, double cost, int frameIndex = -1, int totalFrames = -1)
    {
        lock (_lock)
        {
            Console.SetCursorPosition(0, 0);

            var sb = new StringBuilder(2000);

            // Header (rows 0-3)
            AppendHeader(sb, iteration, cost, frameIndex, totalFrames);

            // Create animation grid (17 rows × 80 columns)
            var grid = new char[AnimationHeight, Width];
            InitializeGrid(grid);
            DrawTrack(grid);
            DrawCartPole(grid, cartPos, poleAngle);

            // Append grid to output
            AppendGrid(sb, grid);

            // Footer (rows 21-24)
            AppendFooter(sb, cartPos, cartVel, poleAngle, poleVel, force);

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

    private static void AppendHeader(StringBuilder sb, int iteration, double cost, int frameIndex = -1, int totalFrames = -1)
    {
        sb.AppendLine($"╔{'═'.ToString().PadRight(Width - 2, '═')}╗");
        sb.AppendLine($"║ {"CART-POLE STABILIZATION - Inverted Pendulum Balance".PadRight(Width - 4)}║");

        if (frameIndex >= 0 && totalFrames > 0)
        {
            var progress = $"Frame: {frameIndex + 1}/{totalFrames} (0=start)";
            sb.AppendLine($"║ Iteration: {iteration,-10} Cost: {cost:F6}  {progress}{string.Empty.PadRight(Width - 67)}║");
        }
        else
        {
            sb.AppendLine($"║ Iteration: {iteration,-10} Cost: {cost:F6}{string.Empty.PadRight(Width - 40)}║");
        }

        sb.AppendLine($"╚{'═'.ToString().PadRight(Width - 2, '═')}╝");
    }

    private static void AppendFooter(StringBuilder sb, double cartPos, double cartVel, double poleAngle, double poleVel, double force)
    {
        var angleDegrees = poleAngle * 180.0 / Math.PI;

        sb.AppendLine();
        sb.AppendLine($"┌{"─".PadRight(Width - 2, '─')}┐");
        var footerText = $"│ x:{cartPos,5:F2}m ẋ:{cartVel,5:F2}m/s │ θ:{angleDegrees,6:F1}° (0°=up) θ̇:{poleVel,5:F2}r/s │ F:{force,5:F2}N";
        sb.Append(footerText);
        sb.Append(string.Empty.PadRight(Width - footerText.Length - 1));
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

    private static void DrawTrack(char[,] grid)
    {
        var trackRow = AnimationHeight - 3;  // Track near bottom

        // Draw horizontal track
        for (var col = 5; col < Width - 5; col++)
        {
            grid[trackRow, col] = '═';
        }
    }

    private static void DrawCartPole(char[,] grid, double cartPos, double poleAngle)
    {
        var trackRow = AnimationHeight - 3;
        var cartRow = trackRow - 1;  // Cart sits on track

        // Map cart position to screen coordinates
        // Assume cartPos range is roughly [-1, 1] meters
        var centerCol = Width / 2;
        var scale = 20.0;  // pixels per meter
        var cartCol = centerCol + (int)(cartPos * scale);

        // Clamp cart position to visible area
        cartCol = Math.Max(CartWidth / 2 + 5, Math.Min(Width - CartWidth / 2 - 5, cartCol));

        // Draw cart body
        var cartLeft = cartCol - CartWidth / 2;
        var cartRight = cartCol + CartWidth / 2;

        if (InBounds(grid, cartRow, cartLeft) && InBounds(grid, cartRow, cartRight))
        {
            for (var col = cartLeft; col <= cartRight; col++)
            {
                if (InBounds(grid, cartRow, col))
                {
                    grid[cartRow, col] = '█';
                }
            }
        }

        // Draw cart wheels
        if (InBounds(grid, trackRow, cartLeft))
        {
            grid[trackRow, cartLeft] = 'o';
        }
        if (InBounds(grid, trackRow, cartRight))
        {
            grid[trackRow, cartRight] = 'o';
        }

        // Draw pole from cart center
        // Note: In cart-pole convention, theta = 0 is UPRIGHT (inverted), theta = π is DOWN (hanging)
        // In screen coordinates, row increases downward, so we need to negate the cos term
        var poleStartRow = cartRow;
        var poleStartCol = cartCol;

        // Compute pole endpoint
        // theta=0 (upright) should point upward (negative row direction)
        // theta=π (down) should point downward (positive row direction)
        var poleEndCol = poleStartCol + (int)(PoleLength * Math.Sin(poleAngle));
        var poleEndRow = poleStartRow - (int)(PoleLength * Math.Cos(poleAngle));  // Note: minus for upright at θ=0

        // Draw pole
        DrawPole(grid, poleStartRow, poleStartCol, poleEndRow, poleEndCol, poleAngle);

        // Draw pole bob/tip
        if (InBounds(grid, poleEndRow, poleEndCol))
        {
            grid[poleEndRow, poleEndCol] = '●';
        }
    }

    private static void DrawPole(char[,] grid, int row0, int col0, int row1, int col1, double theta)
    {
        var lineChar = GetPoleCharacter(theta);

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
                // Don't overwrite cart
                if (grid[currentRow, currentCol] == ' ' || grid[currentRow, currentCol] == '─')
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

    private static char GetPoleCharacter(double theta)
    {
        // Normalize theta to [-π, π]
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
