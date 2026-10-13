/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Linq;
using System.Reflection;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using OptimalCli.Problems.Brachistochrone;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// The scalar generator's output for every existing <c>[OptimalCode]</c> class, in this project and in OptimalCli, is
    /// byte-identical to the goldens captured from the generator before array kernels were added. Each golden is named
    /// <c>{Project}.{HintName}.golden</c> and is compared with the file the build emitted (EmitCompilerGeneratedFiles).
    /// </summary>
    [TestClass]
    public class GeneratedSourceGoldenTests
    {
        private const string GeneratorDirectory = "Optimal.AutoDiff/Optimal.AutoDiff.Analyzers.OptimalIncrementalSourceGenerator";

        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void ScalarGeneratorOutputIsByteIdenticalToGoldens()
        {
            var goldens = Directory.GetFiles(Metadata("GoldenDirectory"), "*.golden").OrderBy(f => f, StringComparer.Ordinal).ToList();
            var mismatches = goldens.Where(golden => !Matches(golden)).Select(Path.GetFileName).ToList();
            TestContext.WriteLine($"{goldens.Count} goldens compared; {mismatches.Count} differ: {string.Join(", ", mismatches)}");
            Assert.HasCount(23, goldens);
            Assert.IsEmpty(mismatches);
        }

        [TestMethod]
        public void EveryGoldenIsStillGenerated()
        {
            // The emitted-files folder is not cleaned between builds, so a file the generator stopped producing would
            // linger; the type it declares must still be in the built assembly.
            var missing = Directory.GetFiles(Metadata("GoldenDirectory"), "*.golden")
                .Select(Path.GetFileName)
                .Where(name => DeclaredType(name!) == null)
                .ToList();
            Assert.IsEmpty(missing, $"Generated types missing for: {string.Join(", ", missing)}");
        }

        private static bool Matches(string golden)
        {
            var (project, hintName) = Split(Path.GetFileName(golden));
            var emitted = Path.Combine(Metadata(project == "OptimalCli" ? "OptimalCliGeneratedDirectory" : "OptimalTestsGeneratedDirectory"), GeneratorDirectory, hintName);
            return File.Exists(emitted) && File.ReadAllBytes(emitted).AsSpan().SequenceEqual(File.ReadAllBytes(golden));
        }

        [UnconditionalSuppressMessage("Trimming", "IL2026", Justification = "The test assembly is not trimmed.")]
        private static Type? DeclaredType(string goldenName)
        {
            var (project, hintName) = Split(goldenName);
            var assembly = project == "OptimalCli" ? typeof(BrachistochroneDynamics).Assembly : typeof(GeneratedSourceGoldenTests).Assembly;
            var typeName = hintName == "OptimalAttributes.g.cs" ? "OptimalCodeAttribute" : hintName[..^".g.cs".Length];
            return assembly.GetTypes().FirstOrDefault(t => t.Name == typeName);
        }

        // "OptimalCli.CornerDynamicsGradients.g.cs.golden" → ("OptimalCli", "CornerDynamicsGradients.g.cs").
        private static (string Project, string HintName) Split(string goldenName)
        {
            var name = goldenName[..^".golden".Length];
            var project = name.StartsWith("OptimalCli.", StringComparison.Ordinal) ? "OptimalCli" : "Optimal.Tests";
            return (project, name[(project.Length + 1)..]);
        }

        private static string Metadata(string key)
        {
            return typeof(GeneratedSourceGoldenTests).Assembly
                .GetCustomAttributes<AssemblyMetadataAttribute>()
                .Single(a => a.Key == key)
                .Value!;
        }
    }
}
