/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Optimal.Control;

/// <summary>
/// Generates HTML visualizations of CollocationResult for visual verification.
/// </summary>
internal static class ResultVisualizer
{
    /// <summary>
    /// Generates an enhanced HTML visualization specifically for the Brachistochrone problem.
    /// Includes 2D path plot, speed verification, and comparison with straight line.
    /// </summary>
    public static string GenerateBrachistochroneHtml(
        CollocationResult result,
        double g,
        double xFinal,
        double yFinal,
        string? outputPath = null)
    {
        ArgumentNullException.ThrowIfNull(result);

        // Determine output path
        var defaultDir = Path.Combine(
            Directory.GetCurrentDirectory(),
            "visualizations");

        var outputDir = outputPath ?? defaultDir;
        Directory.CreateDirectory(outputDir);

        var fileName = $"Brachistochrone_{DateTime.Now:yyyyMMdd_HHmmss}.html";
        var filePath = Path.Combine(outputDir, fileName);

        // Build HTML with enhanced Brachistochrone-specific visualizations
        var html = BuildBrachistochroneHtmlContent(result, g, xFinal, yFinal);

        File.WriteAllText(filePath, html);

        return filePath;
    }

    private static string BuildBrachistochroneHtmlContent(
        CollocationResult result,
        double g,
        double xFinal,
        double yFinal)
    {
        var sb = new StringBuilder();

        // Extract x, y, v data
        var xData = string.Join(", ", result.States.Select(s => s[0].ToString("F6")));
        var yData = string.Join(", ", result.States.Select(s => s[1].ToString("F6")));
        var vData = string.Join(", ", result.States.Select(s => s[2].ToString("F6")));
        var times = string.Join(", ", result.Times.Select(t => t.ToString("F6")));

        // Theoretical v = sqrt(2g|y|)
        var theoreticalV = string.Join(", ", result.States.Select(s =>
        {
            var y = s[1];
            return Math.Sqrt(2.0 * g * Math.Abs(y)).ToString("F6");
        }));

        // Straight line path
        var straightX = new List<double>();
        var straightY = new List<double>();
        for (var i = 0; i < 50; i++)
        {
            var t = i / 49.0;
            straightX.Add(xFinal * t);
            straightY.Add(yFinal * t);
        }
        var straightXData = string.Join(", ", straightX.Select(x => x.ToString("F6")));
        var straightYData = string.Join(", ", straightY.Select(y => y.ToString("F6")));

        // HTML header
        sb.AppendLine("<!DOCTYPE html>");
        sb.AppendLine("<html lang=\"en\">");
        sb.AppendLine("<head>");
        sb.AppendLine("    <meta charset=\"UTF-8\">");
        sb.AppendLine("    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
        sb.AppendLine("    <title>Brachistochrone Problem - Optimal Control Solution</title>");
        sb.AppendLine("    <script src=\"https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js\"></script>");
        sb.AppendLine("    <style>");
        sb.AppendLine("        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 20px; background: #f5f5f5; }");
        sb.AppendLine("        .container { max-width: 1600px; margin: 0 auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }");
        sb.AppendLine("        h1 { color: #333; border-bottom: 3px solid #4CAF50; padding-bottom: 10px; }");
        sb.AppendLine("        h2 { color: #555; margin-top: 30px; }");
        sb.AppendLine("        .metrics { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 20px 0; }");
        sb.AppendLine("        .metric-card { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }");
        sb.AppendLine("        .metric-card.success { background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%); }");
        sb.AppendLine("        .metric-card.warning { background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%); }");
        sb.AppendLine("        .metric-label { font-size: 0.9em; opacity: 0.9; margin-bottom: 5px; }");
        sb.AppendLine("        .metric-value { font-size: 1.8em; font-weight: bold; }");
        sb.AppendLine("        .chart-container { position: relative; height: 400px; margin: 20px 0; }");
        sb.AppendLine("        .chart-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }");
        sb.AppendLine("        .success-badge { display: inline-block; padding: 5px 15px; background: #4CAF50; color: white; border-radius: 20px; font-weight: bold; }");
        sb.AppendLine("        .failure-badge { display: inline-block; padding: 5px 15px; background: #f44336; color: white; border-radius: 20px; font-weight: bold; }");
        sb.AppendLine("        .info-box { background: #e3f2fd; border-left: 4px solid #2196F3; padding: 15px; margin: 20px 0; border-radius: 4px; }");
        sb.AppendLine("    </style>");
        sb.AppendLine("</head>");
        sb.AppendLine("<body>");
        sb.AppendLine("    <div class=\"container\">");

        // Title and status
        sb.AppendLine("        <h1>Brachistochrone Problem (Johann Bernoulli, 1696)</h1>");
        var statusBadge = result.Success ? "success-badge\">SUCCESS" : "failure-badge\">FAILED";
        sb.AppendLine($"        <p><span class=\"{statusBadge}</span> {result.Message}</p>");

        // Info box
        sb.AppendLine("        <div class=\"info-box\">");
        sb.AppendLine("            <strong>The Challenge:</strong> What curve gives the fastest descent under gravity between two points?<br>");
        sb.AppendLine("            <strong>The Answer:</strong> A cycloid - the curve traced by a point on a rolling circle.<br>");
        sb.AppendLine("            <strong>Physics:</strong> Conservation of energy gives v = √(2gh) where h is vertical drop.");
        sb.AppendLine("        </div>");

        // Metrics
        sb.AppendLine("        <h2>Solution Metrics</h2>");
        sb.AppendLine("        <div class=\"metrics\">");
        sb.AppendLine("            <div class=\"metric-card success\">");
        sb.AppendLine("                <div class=\"metric-label\">Descent Time</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.OptimalCost:F3} s</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("            <div class=\"metric-card\">");
        sb.AppendLine("                <div class=\"metric-label\">Iterations</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.Iterations}</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine($"            <div class=\"metric-card {(result.MaxDefect < 1e-2 ? "success" : "warning")}\">");
        sb.AppendLine("                <div class=\"metric-label\">Max Defect</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.MaxDefect:E3}</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("            <div class=\"metric-card\">");
        sb.AppendLine("                <div class=\"metric-label\">Gradient Norm</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.GradientNorm:E3}</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("        </div>");

        // 2D Path Plot
        sb.AppendLine("        <h2>Descent Path (Curve Shape)</h2>");
        sb.AppendLine("        <div class=\"chart-container\">");
        sb.AppendLine("            <canvas id=\"pathChart\"></canvas>");
        sb.AppendLine("        </div>");

        // Speed verification and time-series in grid
        sb.AppendLine("        <div class=\"chart-grid\">");
        sb.AppendLine("            <div>");
        sb.AppendLine("                <h2>Speed Verification</h2>");
        sb.AppendLine("                <div class=\"chart-container\">");
        sb.AppendLine("                    <canvas id=\"speedChart\"></canvas>");
        sb.AppendLine("                </div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("            <div>");
        sb.AppendLine("                <h2>Speed vs Time</h2>");
        sb.AppendLine("                <div class=\"chart-container\">");
        sb.AppendLine("                    <canvas id=\"speedTimeChart\"></canvas>");
        sb.AppendLine("                </div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("        </div>");

        // JavaScript for charts
        sb.AppendLine("        <script>");

        // 2D Path Chart
        sb.AppendLine("        const pathCtx = document.getElementById('pathChart').getContext('2d');");
        sb.AppendLine("        new Chart(pathCtx, {");
        sb.AppendLine("            type: 'scatter',");
        sb.AppendLine("            data: {");
        sb.AppendLine("                datasets: [{");
        sb.AppendLine("                    label: 'Brachistochrone Path',");
        sb.AppendLine($"                    data: [{xData}].map((x, i) => ({{x: x, y: [{yData}][i]}})),");
        sb.AppendLine("                    borderColor: '#2196F3',");
        sb.AppendLine("                    backgroundColor: '#2196F3',");
        sb.AppendLine("                    showLine: true,");
        sb.AppendLine("                    borderWidth: 3,");
        sb.AppendLine("                    pointRadius: 2");
        sb.AppendLine("                }, {");
        sb.AppendLine("                    label: 'Straight Line',");
        sb.AppendLine($"                    data: [{straightXData}].map((x, i) => ({{x: x, y: [{straightYData}][i]}})),");
        sb.AppendLine("                    borderColor: '#FF6384',");
        sb.AppendLine("                    backgroundColor: '#FF6384',");
        sb.AppendLine("                    showLine: true,");
        sb.AppendLine("                    borderWidth: 2,");
        sb.AppendLine("                    borderDash: [5, 5],");
        sb.AppendLine("                    pointRadius: 0");
        sb.AppendLine("                }]");
        sb.AppendLine("            },");
        sb.AppendLine("            options: {");
        sb.AppendLine("                responsive: true,");
        sb.AppendLine("                maintainAspectRatio: false,");
        sb.AppendLine("                plugins: { legend: { position: 'top' } },");
        sb.AppendLine("                scales: {");
        sb.AppendLine("                    x: { title: { display: true, text: 'Horizontal Position x (m)' } },");
        sb.AppendLine("                    y: { title: { display: true, text: 'Vertical Position y (m)' }, reverse: false }");
        sb.AppendLine("                }");
        sb.AppendLine("            }");
        sb.AppendLine("        });");

        // Speed Verification Chart (v vs y)
        sb.AppendLine();
        sb.AppendLine("        const speedCtx = document.getElementById('speedChart').getContext('2d');");
        sb.AppendLine("        new Chart(speedCtx, {");
        sb.AppendLine("            type: 'scatter',");
        sb.AppendLine("            data: {");
        sb.AppendLine("                datasets: [{");
        sb.AppendLine("                    label: 'Actual Speed',");
        sb.AppendLine($"                    data: [{yData}].map((y, i) => ({{x: y, y: [{vData}][i]}})),");
        sb.AppendLine("                    borderColor: '#4CAF50',");
        sb.AppendLine("                    backgroundColor: '#4CAF50',");
        sb.AppendLine("                    showLine: true,");
        sb.AppendLine("                    borderWidth: 2,");
        sb.AppendLine("                    pointRadius: 2");
        sb.AppendLine("                }, {");
        sb.AppendLine("                    label: 'Theory: v = √(2g|y|)',");
        sb.AppendLine($"                    data: [{yData}].map((y, i) => ({{x: y, y: [{theoreticalV}][i]}})),");
        sb.AppendLine("                    borderColor: '#FF9800',");
        sb.AppendLine("                    backgroundColor: '#FF9800',");
        sb.AppendLine("                    showLine: true,");
        sb.AppendLine("                    borderWidth: 2,");
        sb.AppendLine("                    borderDash: [5, 5],");
        sb.AppendLine("                    pointRadius: 0");
        sb.AppendLine("                }]");
        sb.AppendLine("            },");
        sb.AppendLine("            options: {");
        sb.AppendLine("                responsive: true,");
        sb.AppendLine("                maintainAspectRatio: false,");
        sb.AppendLine("                plugins: { legend: { position: 'top' } },");
        sb.AppendLine("                scales: {");
        sb.AppendLine("                    x: { title: { display: true, text: 'Vertical Position y (m)' } },");
        sb.AppendLine("                    y: { title: { display: true, text: 'Speed v (m/s)' } }");
        sb.AppendLine("                }");
        sb.AppendLine("            }");
        sb.AppendLine("        });");

        // Speed vs Time Chart
        sb.AppendLine();
        sb.AppendLine("        const speedTimeCtx = document.getElementById('speedTimeChart').getContext('2d');");
        sb.AppendLine("        new Chart(speedTimeCtx, {");
        sb.AppendLine("            type: 'line',");
        sb.AppendLine("            data: {");
        sb.AppendLine($"                labels: [{times}],");
        sb.AppendLine("                datasets: [{");
        sb.AppendLine("                    label: 'Speed v(t)',");
        sb.AppendLine($"                    data: [{vData}],");
        sb.AppendLine("                    borderColor: '#9C27B0',");
        sb.AppendLine("                    backgroundColor: '#9C27B033',");
        sb.AppendLine("                    borderWidth: 2,");
        sb.AppendLine("                    fill: false,");
        sb.AppendLine("                    tension: 0.4");
        sb.AppendLine("                }]");
        sb.AppendLine("            },");
        sb.AppendLine("            options: {");
        sb.AppendLine("                responsive: true,");
        sb.AppendLine("                maintainAspectRatio: false,");
        sb.AppendLine("                plugins: { legend: { position: 'top' } },");
        sb.AppendLine("                scales: {");
        sb.AppendLine("                    x: { title: { display: true, text: 'Time (s)' } },");
        sb.AppendLine("                    y: { title: { display: true, text: 'Speed (m/s)' } }");
        sb.AppendLine("                }");
        sb.AppendLine("            }");
        sb.AppendLine("        });");

        sb.AppendLine("        </script>");
        sb.AppendLine("    </div>");
        sb.AppendLine("</body>");
        sb.AppendLine("</html>");

        return sb.ToString();
    }

    /// <summary>
    /// Generates a generic HTML visualization for any optimal control problem.
    /// </summary>
    public static string GenerateHtml(
        CollocationResult result,
        string problemName,
        string[] stateLabels,
        string[] controlLabels,
        string? outputPath = null)
    {
        ArgumentNullException.ThrowIfNull(result);
        ArgumentNullException.ThrowIfNull(problemName);
        ArgumentNullException.ThrowIfNull(stateLabels);
        ArgumentNullException.ThrowIfNull(controlLabels);

        // Determine output path
        var defaultDir = Path.Combine(
            Directory.GetCurrentDirectory(),
            "visualizations");

        var outputDir = outputPath ?? defaultDir;
        Directory.CreateDirectory(outputDir);

        var fileName = $"{problemName.Replace(" ", "_")}_{DateTime.Now:yyyyMMdd_HHmmss}.html";
        var filePath = Path.Combine(outputDir, fileName);

        // Build HTML with generic visualizations
        var html = BuildGenericHtmlContent(result, problemName, stateLabels, controlLabels);

        File.WriteAllText(filePath, html);

        return filePath;
    }

    private static string BuildGenericHtmlContent(
        CollocationResult result,
        string problemName,
        string[] stateLabels,
        string[] controlLabels)
    {
        var sb = new StringBuilder();

        var times = string.Join(", ", result.Times.Select(t => t.ToString("F6")));

        // HTML header
        sb.AppendLine("<!DOCTYPE html>");
        sb.AppendLine("<html lang=\"en\">");
        sb.AppendLine("<head>");
        sb.AppendLine("    <meta charset=\"UTF-8\">");
        sb.AppendLine("    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
        sb.AppendLine($"    <title>{problemName} - Optimal Control Solution</title>");
        sb.AppendLine("    <script src=\"https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js\"></script>");
        sb.AppendLine("    <style>");
        sb.AppendLine("        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 20px; background: #f5f5f5; }");
        sb.AppendLine("        .container { max-width: 1600px; margin: 0 auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }");
        sb.AppendLine("        h1 { color: #333; border-bottom: 3px solid #4CAF50; padding-bottom: 10px; }");
        sb.AppendLine("        h2 { color: #555; margin-top: 30px; }");
        sb.AppendLine("        .metrics { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 20px 0; }");
        sb.AppendLine("        .metric-card { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }");
        sb.AppendLine("        .metric-card.success { background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%); }");
        sb.AppendLine("        .metric-card.warning { background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%); }");
        sb.AppendLine("        .metric-label { font-size: 0.9em; opacity: 0.9; margin-bottom: 5px; }");
        sb.AppendLine("        .metric-value { font-size: 1.8em; font-weight: bold; }");
        sb.AppendLine("        .chart-container { position: relative; height: 400px; margin: 20px 0; }");
        sb.AppendLine("        .success-badge { display: inline-block; padding: 5px 15px; background: #4CAF50; color: white; border-radius: 20px; font-weight: bold; }");
        sb.AppendLine("        .failure-badge { display: inline-block; padding: 5px 15px; background: #f44336; color: white; border-radius: 20px; font-weight: bold; }");
        sb.AppendLine("    </style>");
        sb.AppendLine("</head>");
        sb.AppendLine("<body>");
        sb.AppendLine("    <div class=\"container\">");

        // Title and status
        sb.AppendLine($"        <h1>{problemName}</h1>");
        var statusBadge = result.Success ? "success-badge\">SUCCESS" : "failure-badge\">FAILED";
        sb.AppendLine($"        <p><span class=\"{statusBadge}</span> {result.Message}</p>");

        // Metrics
        sb.AppendLine("        <h2>Solution Metrics</h2>");
        sb.AppendLine("        <div class=\"metrics\">");
        sb.AppendLine("            <div class=\"metric-card success\">");
        sb.AppendLine("                <div class=\"metric-label\">Optimal Cost</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.OptimalCost:E3}</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("            <div class=\"metric-card\">");
        sb.AppendLine("                <div class=\"metric-label\">Iterations</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.Iterations}</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine($"            <div class=\"metric-card {(result.MaxDefect < 1e-2 ? "success" : "warning")}\">");
        sb.AppendLine("                <div class=\"metric-label\">Max Defect</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.MaxDefect:E3}</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("            <div class=\"metric-card\">");
        sb.AppendLine("                <div class=\"metric-label\">Gradient Norm</div>");
        sb.AppendLine($"                <div class=\"metric-value\">{result.GradientNorm:E3}</div>");
        sb.AppendLine("            </div>");
        sb.AppendLine("        </div>");

        // State trajectories
        sb.AppendLine("        <h2>State Trajectories</h2>");
        sb.AppendLine("        <div class=\"chart-container\">");
        sb.AppendLine("            <canvas id=\"stateChart\"></canvas>");
        sb.AppendLine("        </div>");

        // Control trajectories
        sb.AppendLine("        <h2>Control Trajectories</h2>");
        sb.AppendLine("        <div class=\"chart-container\">");
        sb.AppendLine("            <canvas id=\"controlChart\"></canvas>");
        sb.AppendLine("        </div>");

        // JavaScript for charts
        sb.AppendLine("        <script>");

        // State chart
        sb.AppendLine("        const stateCtx = document.getElementById('stateChart').getContext('2d');");
        sb.AppendLine("        new Chart(stateCtx, {");
        sb.AppendLine("            type: 'line',");
        sb.AppendLine("            data: {");
        sb.AppendLine($"                labels: [{times}],");
        sb.AppendLine("                datasets: [");

        var colors = new[] { "#2196F3", "#4CAF50", "#FF9800", "#9C27B0", "#F44336", "#00BCD4" };
        for (var i = 0; i < stateLabels.Length; i++)
        {
            var stateData = string.Join(", ", result.States.Select(s => s[i].ToString("F6")));
            var color = colors[i % colors.Length];
            sb.AppendLine("                    {");
            sb.AppendLine($"                        label: '{stateLabels[i]}',");
            sb.AppendLine($"                        data: [{stateData}],");
            sb.AppendLine($"                        borderColor: '{color}',");
            sb.AppendLine($"                        backgroundColor: '{color}33',");
            sb.AppendLine("                        borderWidth: 2,");
            sb.AppendLine("                        fill: false,");
            sb.AppendLine("                        tension: 0.4");
            sb.AppendLine(i < stateLabels.Length - 1 ? "                    }," : "                    }");
        }

        sb.AppendLine("                ]");
        sb.AppendLine("            },");
        sb.AppendLine("            options: {");
        sb.AppendLine("                responsive: true,");
        sb.AppendLine("                maintainAspectRatio: false,");
        sb.AppendLine("                plugins: { legend: { position: 'top' } },");
        sb.AppendLine("                scales: {");
        sb.AppendLine("                    x: { title: { display: true, text: 'Time (s)' } },");
        sb.AppendLine("                    y: { title: { display: true, text: 'State Values' } }");
        sb.AppendLine("                }");
        sb.AppendLine("            }");
        sb.AppendLine("        });");

        // Control chart
        sb.AppendLine();
        sb.AppendLine("        const controlCtx = document.getElementById('controlChart').getContext('2d');");
        sb.AppendLine("        new Chart(controlCtx, {");
        sb.AppendLine("            type: 'line',");
        sb.AppendLine("            data: {");
        sb.AppendLine($"                labels: [{times}],");
        sb.AppendLine("                datasets: [");

        for (var i = 0; i < controlLabels.Length; i++)
        {
            var controlData = string.Join(", ", result.Controls.Select(c => c[i].ToString("F6")));
            var color = colors[i % colors.Length];
            sb.AppendLine("                    {");
            sb.AppendLine($"                        label: '{controlLabels[i]}',");
            sb.AppendLine($"                        data: [{controlData}],");
            sb.AppendLine($"                        borderColor: '{color}',");
            sb.AppendLine($"                        backgroundColor: '{color}33',");
            sb.AppendLine("                        borderWidth: 2,");
            sb.AppendLine("                        fill: false,");
            sb.AppendLine("                        tension: 0.4,");
            sb.AppendLine("                        stepped: 'before'");
            sb.AppendLine(i < controlLabels.Length - 1 ? "                    }," : "                    }");
        }

        sb.AppendLine("                ]");
        sb.AppendLine("            },");
        sb.AppendLine("            options: {");
        sb.AppendLine("                responsive: true,");
        sb.AppendLine("                maintainAspectRatio: false,");
        sb.AppendLine("                plugins: { legend: { position: 'top' } },");
        sb.AppendLine("                scales: {");
        sb.AppendLine("                    x: { title: { display: true, text: 'Time (s)' } },");
        sb.AppendLine("                    y: { title: { display: true, text: 'Control Values' } }");
        sb.AppendLine("                }");
        sb.AppendLine("            }");
        sb.AppendLine("        });");

        sb.AppendLine("        </script>");
        sb.AppendLine("    </div>");
        sb.AppendLine("</body>");
        sb.AppendLine("</html>");

        return sb.ToString();
    }
}
