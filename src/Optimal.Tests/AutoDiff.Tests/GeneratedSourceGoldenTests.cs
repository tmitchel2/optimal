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
using System.Security.Cryptography;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using OptimalCli.Problems.Brachistochrone;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// The scalar generator's output for every existing <c>[OptimalCode]</c> class, in this project and in OptimalCli, is
    /// byte-identical to the goldens captured from the generator before array kernels were added. Each golden is named
    /// <c>{Project}.{HintName}.golden</c> and is compared with the file the build emitted (EmitCompilerGeneratedFiles). The
    /// kernel generator's files are checked against recorded hashes.
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

        [TestMethod]
        public void KernelGeneratorOutputMatchesItsRecordedHashes()
        {
            // The kernel files are megabytes, so their goldens are SHA-256 hashes of the text with line endings as
            // "\n": a line "<hash> <hint name>" each. A deliberate change to the generator updates the hash with it.
            var manifest = File.ReadAllLines(Path.Combine(Metadata("GoldenDirectory"), "Kernels.sha256"))
                .Where(line => line.Length > 0 && !line.StartsWith('#'))
                .Select(line => line.Split(' ', 2, StringSplitOptions.RemoveEmptyEntries))
                .ToList();
            var directory = Path.Combine(Metadata("OptimalTestsGeneratedDirectory"), GeneratorDirectory);
            var mismatches = manifest
                .Where(entry => !File.Exists(Path.Combine(directory, entry[1])) || Hash(Path.Combine(directory, entry[1])) != entry[0])
                .Select(entry => $"{entry[1]} ({(File.Exists(Path.Combine(directory, entry[1])) ? Hash(Path.Combine(directory, entry[1])) : "missing")})")
                .ToList();
            TestContext.WriteLine($"{manifest.Count} kernel files hashed; {mismatches.Count} differ: {string.Join(", ", mismatches)}");
            Assert.HasCount(9, manifest);
            Assert.IsEmpty(mismatches);
        }

        private static string Hash(string path)
        {
            var text = File.ReadAllText(path).Replace("\r\n", "\n", StringComparison.Ordinal);
            return Convert.ToHexStringLower(SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(text)));
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
