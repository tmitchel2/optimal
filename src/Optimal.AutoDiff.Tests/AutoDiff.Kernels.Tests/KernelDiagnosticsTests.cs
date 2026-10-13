/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Linq;
using Microsoft.CodeAnalysis;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Analyzers.Kernels.Tests.KernelSource;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>Each generator diagnostic, raised by a minimal kernel, with its message and the line it points at.</summary>
    [TestClass]
    public class KernelDiagnosticsTests
    {
        [TestMethod]
        public void DataDependentIfIsOptad100()
        {
            var source = Kernel("""
                if (x[0] > 0) // here
                {
                    y[0] = x[0];
                }
                else
                {
                    y[0] = -x[0];
                }

                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD100", "if statement's condition depends on run-time values", "// here");
        }

        [TestMethod]
        public void DataDependentLoopBoundIsOptad100()
        {
            var source = Kernel("""
                y[0] = 0;
                for (var i = 0; i < x[0]; i++) // here
                {
                    y[0] += x[1];
                }

                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD100", "loop condition depends on run-time values", "// here");
        }

        [TestMethod]
        public void DataDependentIndexIsOptad100()
        {
            var source = Kernel("""
                y[0] = x[(int)x[1]]; // here
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD100", "run-time double is converted to int", "// here");
        }

        [TestMethod]
        public void RunTimeConditionsJoinedByAndAreOptad100()
        {
            var source = Kernel("""
                y[0] = x[0] > 0 && x[1] > 0 ? x[2] : 0; // here
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD100", "&& and || need compile-time operands", "// here");
        }

        [TestMethod]
        public void ForeachIsOptad101()
        {
            var source = Kernel("""
                y[1] = 0;
                foreach (var v in x) // here
                {
                    y[0] = v;
                }
                """);
            AssertDiagnostic(source, "OPTAD101", "the statement 'foreach (var v in x)", "// here");
        }

        [TestMethod]
        public void IncrementInsideAnExpressionIsOptad101()
        {
            var source = Kernel("""
                var i = 0;
                y[i++] = x[0]; // here
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD101", "is not supported", "// here");
        }

        [TestMethod]
        public void LocalFunctionIsOptad101()
        {
            var source = Kernel("""
                double Square(double v) => v * v; // here
                y[0] = Square(x[0]);
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD101", "local functions are not supported", "// here");
        }

        [TestMethod]
        public void SpanWithoutFixedLengthIsOptad102()
        {
            var source = Kernel(
                "y[0] = x[0]; y[1] = 0;",
                signature: "ReadOnlySpan<double> x, [FixedLength(2)] Span<double> y");
            AssertDiagnostic(source, "OPTAD102", "span parameter 'x' needs [FixedLength(n)]", "public static void F");
        }

        [TestMethod]
        public void TwoOutputsAreOptad102()
        {
            var source = Kernel(
                "y[0] = x[0]; y[1] = 0; z[0] = 0;",
                signature: "[FixedLength(3)] ReadOnlySpan<double> x, [FixedLength(2)] Span<double> y, [FixedLength(1)] Span<double> z");
            AssertDiagnostic(source, "OPTAD102", "exactly one Span<double> output, and this one has 2", "public static void F");
        }

        [TestMethod]
        public void IntParameterIsOptad102()
        {
            var source = Kernel(
                "y[0] = x[0]; y[1] = n;",
                signature: "[FixedLength(3)] ReadOnlySpan<double> x, int n, [FixedLength(2)] Span<double> y");
            AssertDiagnostic(source, "OPTAD102", "parameter 'n' has type int", "public static void F");
        }

        [TestMethod]
        public void ReservedParameterNameIsOptad102()
        {
            var source = Kernel(
                "y[0] = v1; y[1] = 0;",
                signature: "[FixedLength(3)] ReadOnlySpan<double> x, double v1, [FixedLength(2)] Span<double> y");
            AssertDiagnostic(source, "OPTAD102", "the parameter name 'v1' is reserved", "public static void F");
        }

        [TestMethod]
        public void UnsupportedMathFunctionIsOptad103()
        {
            var source = Kernel("""
                y[0] = Math.Floor(x[0]); // here
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD103", "Math.Floor on double is not supported in a kernel; the supported functions are", "// here");
        }

        [TestMethod]
        public void HelperOutsideOptimalCodeIsOptad103()
        {
            var source = Kernel(
                """
                y[0] = Plain.Square(x[0]); // here
                y[1] = 0;
                """,
                types: "    public static class Plain\n    {\n        public static double Square(double v) => v * v;\n    }");
            AssertDiagnostic(source, "OPTAD103", "'Probes.Plain.Square' cannot be inlined", "// here");
        }

        [TestMethod]
        public void OutOfRangeIndexIsOptad104()
        {
            var source = Kernel("""
                y[0] = x[3]; // here
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD104", "the index 3 is outside 'x', whose length is 3", "// here");
        }

        [TestMethod]
        public void OutOfRangeSliceIsOptad104()
        {
            var source = Kernel("""
                var tail = x.Slice(2, 2); // here
                y[0] = tail[0];
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD104", "the slice (2, 2) is outside 'x'", "// here");
        }

        [TestMethod]
        public void UnwrittenOutputIsOptad105()
        {
            var source = Kernel("y[0] = x[0];");
            AssertDiagnostic(source, "OPTAD105", "the output 'y[1]' is never written", "public static void F");
        }

        [TestMethod]
        public void OutputReadBeforeWriteIsOptad105()
        {
            var source = Kernel("""
                y[0] = y[1] * x[0]; // here
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD105", "'y[1]' is read before it is written", "// here");
        }

        [TestMethod]
        public void RunawayLoopIsOptad106()
        {
            var source = Kernel("""
                y[0] = 0;
                while (true) // here
                {
                    y[0] += x[0];
                }
                """);
            AssertDiagnostic(source, "OPTAD106", "more than 1,000,000 loop iterations", "// here");
        }

        [TestMethod]
        public void UnboundedRecursionIsOptad106()
        {
            var source = Kernel(
                """
                y[0] = Deep(x[0], 0); // here
                y[1] = 0;
                """,
                members: "        private static double Deep(double v, int n) => n > 1000 ? v : Deep(v, n + 1);");
            AssertDiagnostic(source, "OPTAD106", "helper calls nest more than 64 deep", "Deep(v, n + 1)");
        }

        [TestMethod]
        public void UnknownWrtIsOptad107()
        {
            var source = Kernel("y[0] = x[0]; y[1] = 0;", attributes: "[Jacobian(\"q\")]");
            AssertDiagnostic(source, "OPTAD107", "'q' is not an input of the kernel", "[Jacobian(\"q\")]");
        }

        [TestMethod]
        public void OutputAsWrtIsOptad107()
        {
            var source = Kernel("y[0] = x[0]; y[1] = 0;", attributes: "[Vjp(nameof(y))]");
            AssertDiagnostic(source, "OPTAD107", "'y' is not an input of the kernel", "[Vjp(nameof(y))]");
        }

        [TestMethod]
        public void RepeatedRequestIsOptad107()
        {
            var source = Kernel("y[0] = x[0]; y[1] = 0;", attributes: "[Jacobian(nameof(x))]\n        [Jacobian(nameof(x), ChunkSize = 10)]");
            AssertDiagnostic(source, "OPTAD107", "'FJacobian_x' is requested twice", "ChunkSize = 10");
        }

        [TestMethod]
        public void ZeroChunkSizeIsOptad107()
        {
            var source = Kernel("y[0] = x[0]; y[1] = 0;", attributes: "[Jacobian(nameof(x), ChunkSize = 0)]");
            AssertDiagnostic(source, "OPTAD107", "ChunkSize is 0", "ChunkSize = 0");
        }

        [TestMethod]
        public void RunTimeExponentIsOptad108()
        {
            var source = Kernel("""
                y[0] = Math.Pow(x[0], x[1]); // here
                y[1] = 0;
                """);
            AssertDiagnostic(source, "OPTAD108", "Math.Pow's exponent depends on run-time values", "// here");
        }

        [TestMethod]
        public void WriteToStaticTableIsOptad109()
        {
            var source = Kernel(
                """
                Table[0] = 2; // here
                y[0] = x[Table[0]];
                y[1] = 0;
                """,
                members: "        private static readonly int[] Table = { 0, 1 };");
            AssertDiagnostic(source, "OPTAD109", "'Table' is read-only to a kernel", "// here");
        }

        [TestMethod]
        public void FailingKernelLeavesItsSiblingsGenerated()
        {
            var source = Kernel(
                "y[0] = x[3]; y[1] = 0;",
                members: """
                        [Jacobian(nameof(x))]
                        public static void G([FixedLength(3)] ReadOnlySpan<double> x, [FixedLength(1)] Span<double> y)
                        {
                            y[0] = x[0] * x[1];
                        }
                """);
            var run = GeneratorHarness.Run(source);
            Assert.AreEqual("OPTAD104", run.Diagnostics.Single().Id);
            var generated = run.Sources["Probes.ProbeKernels.g.cs"];
            Assert.Contains("public static void GJacobian_x(", generated);
            Assert.DoesNotContain("FJacobian_x", generated);
        }

        private static void AssertDiagnostic(string source, string id, string message, string marker)
        {
            var run = GeneratorHarness.Run(source);
            var diagnostic = run.Diagnostics.Single();
            Assert.AreEqual(id, diagnostic.Id, diagnostic.ToString());
            Assert.AreEqual(DiagnosticSeverity.Error, diagnostic.Severity);
            Assert.StartsWith("Kernel '", diagnostic.GetMessage(), diagnostic.ToString());
            Assert.Contains(message, diagnostic.GetMessage(), diagnostic.ToString());
            Assert.AreEqual(GeneratorHarness.LineOf(source, marker), diagnostic.Location.GetLineSpan().StartLinePosition.Line, diagnostic.ToString());
        }
    }
}
