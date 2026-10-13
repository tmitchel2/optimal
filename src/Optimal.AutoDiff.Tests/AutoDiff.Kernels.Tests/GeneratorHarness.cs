/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Immutable;
using System.IO;
using System.Linq;
using System.Reflection;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>Runs the generator in-process on source text, and compiles and loads what it produced.</summary>
    public static class GeneratorHarness
    {
        private static readonly Lazy<ImmutableArray<MetadataReference>> References = new(() =>
            ((string)AppContext.GetData("TRUSTED_PLATFORM_ASSEMBLIES")!)
                .Split(Path.PathSeparator)
                .Where(path => path.EndsWith(".dll", StringComparison.Ordinal))
                .Select(path => (MetadataReference)MetadataReference.CreateFromFile(path))
                .ToImmutableArray());

        /// <summary>The generator's diagnostics, the sources it added, and the compilation with them.</summary>
        public static GeneratorRun Run(string source)
        {
            var compilation = CSharpCompilation.Create(
                "Kernels" + Guid.NewGuid().ToString("N"),
                [CSharpSyntaxTree.ParseText(source, new CSharpParseOptions(LanguageVersion.Latest))],
                References.Value,
                new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary, nullableContextOptions: NullableContextOptions.Enable));
            var driver = CSharpGeneratorDriver.Create(new OptimalIncrementalSourceGenerator())
                .RunGeneratorsAndUpdateCompilation(compilation, out var output, out _);
            var result = driver.GetRunResult().Results.Single();
            Assert.IsNull(result.Exception, $"The generator threw: {result.Exception}");
            return new GeneratorRun(result.Diagnostics, result.GeneratedSources.ToImmutableDictionary(s => s.HintName, s => s.SourceText.ToString()), output);
        }

        /// <summary>Runs the generator, requires it to report nothing, and loads the compiled assembly.</summary>
        public static Assembly Compile(string source)
        {
            var run = Run(source);
            Assert.IsEmpty(run.Diagnostics, string.Join(Environment.NewLine, run.Diagnostics));
            using var stream = new MemoryStream();
            var emitted = run.Output.Emit(stream);
            var errors = emitted.Diagnostics.Where(d => d.Severity == DiagnosticSeverity.Error).ToList();
            Assert.IsEmpty(errors, string.Join(Environment.NewLine, errors));
            return Assembly.Load(stream.ToArray());
        }

        /// <summary>Calls <c>static void {method}(double[] …)</c> of <paramref name="type"/>, a wrapper the test source declares.</summary>
        public static void Invoke(Assembly assembly, string type, string method, params object[] arguments)
        {
            _ = assembly.GetType(type, throwOnError: true)!.GetMethod(method)!.Invoke(null, arguments);
        }

        /// <summary>The zero-based line of the first line of <paramref name="source"/> containing <paramref name="marker"/>.</summary>
        public static int LineOf(string source, string marker)
        {
            var lines = source.Split('\n');
            var line = Array.FindIndex(lines, l => l.Contains(marker, StringComparison.Ordinal));
            Assert.IsGreaterThanOrEqualTo(0, line, $"No line contains '{marker}'.");
            return line;
        }
    }
}
