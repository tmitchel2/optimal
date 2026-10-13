# Optimal

A .NET 10.0 / C# library for **trajectory optimisation** and **nonlinear programming**. Optimal provides a set of composable solvers for classical optimal-control problems — the kind of problems that arise in robotics, aerospace, racing-line design, and process control — together with the supporting nonlinear-optimisation machinery (L-BFGS, L-BFGS-B, Augmented Lagrangian, Conjugate Gradient), a compile-time automatic-differentiation source generator, and a command-line runner that ships with a gallery of classic benchmark problems.

## Highlights

- **Direct transcription solvers** — Hermite–Simpson and Legendre–Gauss–Lobatto collocation, with optional mesh refinement, warm starts, and parallel variants.
- **Indirect solvers** — Pontryagin's Minimum Principle and multiple-shooting variants for problems where costate equations are tractable.
- **Multi-phase problems** — solve problems with phase transitions (e.g. Goddard rocket stages).
- **Nonlinear optimisers** — L-BFGS, L-BFGS-B (bounded), Augmented Lagrangian (general constraints), Conjugate Gradient, Gradient Descent, with pluggable preconditioners and line-search strategies.
- **Automatic differentiation** — a Roslyn source generator (`Optimal.AutoDiff`) produces forward- and reverse-mode derivative code at compile time, avoiding runtime expression-tree overhead.
- **Problem scaling & continuation** — tooling for variable scaling, homotopy/continuation, and initial-guess construction.
- **Monitoring & diagnostics** — gradient verification, smoothness tests, conditioning estimates, convergence monitoring, and line-search instrumentation.
- **SIMD-accelerated** vector operations via `System.Numerics.Tensors`.
- **Strict quality gates** — nullable reference types, warnings-as-errors, and Roslynator analyzers across the solution.

## Repository layout

```
optimal/
├── src/
│   ├── Optimal.slnx                  Solution file
│   ├── Directory.Build.props         Shared MSBuild settings & analyzers
│   ├── Optimal/                      Core library
│   │   ├── Control.Core/             Problem model, solver contracts
│   │   ├── Control.Collocation/      Hermite–Simpson, LGL, mesh refinement
│   │   ├── Control.Solvers/          High-level solver entry points
│   │   ├── Control.Indirect/         Pontryagin / multiple shooting
│   │   ├── Control.Optimization/     Objective/constraint wiring, gradient helpers
│   │   ├── Control.Scaling/          Variable scaling
│   │   ├── Control.Initialization/   Initial-guess construction
│   │   ├── Control.Visualization/    State-to-position mapping for viz
│   │   ├── NonLinear/                Shared NLP components (CG, preconditioners, L-BFGS memory)
│   │   ├── NonLinear.Unconstrained/  L-BFGS
│   │   ├── NonLinear.Constrained/    L-BFGS-B, Augmented Lagrangian
│   │   ├── NonLinear.Constraints/    Constraint abstractions
│   │   ├── NonLinear.LineSearch/     Backtracking & parallel variants
│   │   └── NonLinear.Monitoring/     Gradient verification, conditioning, convergence
│   ├── Optimal.AutoDiff/             Compile-time AD source generator
│   │   ├── AutoDiff.IR/              AST → IR lowering
│   │   ├── AutoDiff.Differentiation/ Forward-mode rules
│   │   ├── AutoDiff.CodeGen/         Forward & reverse-mode emitters
│   │   └── AutoDiff.Analyzers/       Roslyn incremental generator
│   ├── OptimalCli/                   Benchmark problem runner
│   └── Optimal.Tests/                xUnit test suites (one per module)
├── utils/                            Developer shell scripts (build, test, coverage, issues)
├── lib/radiant/                      Visualisation submodule (github.com/tmitchel2/radiant)
├── .devcontainer/                    Dev container for reproducible builds
├── .editorconfig                     Formatting rules
├── global.json                       .NET SDK pinning (10.0.100, latestFeature)
└── CLAUDE.md                         Instructions for Claude Code agent work
```

## Prerequisites

- **.NET SDK 10.0.100** or newer (pinned via `global.json`, roll-forward `latestFeature`).
- A platform supported by .NET 10 (Windows, macOS, Linux).
- Git — the repository uses a submodule (`lib/radiant`) for visualisation.
- **Bash** if you intend to use the helper scripts in `utils/` and `src/build.sh`.

> A `.devcontainer/` is provided for a fully-configured VS Code / Codespaces environment.

## Getting the source

```bash
git clone --recurse-submodules https://github.com/tmitchel2/optimal.git
cd optimal
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Building

```bash
dotnet build src/Optimal.slnx
```

or using the helper:

```bash
./utils/run-build.sh
```

The build treats warnings as errors and runs Roslynator analyzers — a clean build is the standard.

## Running the benchmark CLI

`OptimalCli` ships with a gallery of classic optimal-control problems. From the repository root:

```bash
dotnet run --project src/OptimalCli -- <problem> [options]
```

Available problems:

| Command              | Description                                                                   |
| -------------------- | ----------------------------------------------------------------------------- |
| `brachistochrone`    | Curve of fastest descent under gravity (Johann Bernoulli, 1696)              |
| `brachistochrone-alt`| Brachistochrone with arc-length parameterisation                              |
| `vanderpol`          | Van der Pol oscillator stabilisation with minimum control effort              |
| `pendulum`           | Pendulum swing-up from hanging to inverted                                    |
| `cartpole`           | Cart-pole stabilisation with full nonlinear dynamics                          |
| `dubins`             | Dubins car minimum path with curvature constraint                             |
| `goddard`            | Goddard rocket maximum altitude with drag and fuel constraints                |
| `corner`             | Optimal racing line through a 90° corner with road constraints                |

Common options:

- `--headless`, `-H` — skip visualisation windows (useful for CI / SSH).
- `--variant <type>`, `-v <type>` — select a problem variant (see `--help`).
- `--debug`, `-d` — enable verbose visualisation diagnostics.

Examples:

```bash
dotnet run --project src/OptimalCli -- brachistochrone
dotnet run --project src/OptimalCli -- brachistochrone -v fixed
dotnet run --project src/OptimalCli -- cartpole --headless
dotnet run --project src/OptimalCli -- goddard -v free-tf
```

Run `dotnet run --project src/OptimalCli -- --help` for the full option listing, including per-problem variants.

## Using the library

Add a project reference to `Optimal`:

```xml
<ItemGroup>
  <ProjectReference Include="path/to/Optimal.csproj" />
</ItemGroup>
```

A minimal collocation solve looks like:

```csharp
using Optimal.Control.Core;
using Optimal.Control.Collocation;
using Optimal.Control.Solvers;

var problem = new ControlProblem()
    .WithStateDim(2)
    .WithControlDim(1)
    .WithTimeInterval(0.0, 1.0)
    .WithInitialState([0.0, 0.0])
    .WithFinalState([1.0, 0.0])
    .WithDynamics(input =>
    {
        // ẋ = f(x, u, t)  — return value + Jacobians w.r.t. x and u
        // ...
    })
    .WithRunningCost(input =>
    {
        // L(x, u, t) — return value + gradients
        // ...
    });

var solver = new HermiteSimpsonSolver(new HermiteSimpsonSolverOptions { NumIntervals = 40 });
var result  = solver.Solve(problem);
```

For unconstrained NLP only, use `LBFGSOptimizer`; for bound-constrained problems `LBFGSBOptimizer`; for general nonlinear constraints `AugmentedLagrangianOptimizer`. See the `src/Optimal.Tests/` suites — each solver has a dedicated test project that doubles as usage documentation.

## Testing

Run every test project:

```bash
./utils/run-tests.sh
```

or directly:

```bash
dotnet test src/Optimal.slnx
```

Integration tests are tagged `TestCategory=Integration` and excluded from the default run. To produce a coverage report:

```bash
./src/build.sh
```

This runs the suite with `XPlat Code Coverage`, merges the Cobertura output, and generates an HTML report under `src/bin/test/results/codecoverage/index.html`.

## Developer utilities

The `utils/` directory contains shell helpers used both interactively and by the Claude Code agent workflows in `.claude/`:

- `run-build.sh` / `run-tests.sh` — build / test the solution.
- `get-solution-coverage.sh` — solution-wide line/branch coverage as JSON.
- `get-project-coverage.sh <path>` — per-class coverage for a given project.
- `get-file-coverage.sh <path>` — per-method / per-line coverage for a file.
- `get-least-covered-files.sh` — rank files by uncovered lines.
- `get-crap-scores.sh` — compute CRAP scores to identify risky, poorly-tested code.
- `get-project-dependency-order.sh` — topological order of projects for staged work.
- `create-unit-test-issues.sh` — open GitHub issues for the least-covered files.
- `begin-issue.sh` — pick up an issue and start a branch.
- `help.sh` — list all helpers.

## Automatic differentiation

`Optimal.AutoDiff` is a Roslyn **incremental source generator**. It analyses methods marked for differentiation, lowers them to a small IR (`AutoDiff.IR`), applies forward- or reverse-mode differentiation rules (`AutoDiff.Differentiation`), and emits strongly-typed derivative code at compile time (`AutoDiff.CodeGen`). Consumers get zero-allocation derivative evaluation with no runtime expression-tree overhead.

The generator is wired in via `Optimal.csproj`:

```xml
<ProjectReference Include="..\Optimal.AutoDiff\Optimal.AutoDiff.csproj"
                  ReferenceOutputAssembly="false"
                  OutputItemType="Analyzer" />
```

### Array kernels

A static void method of an `[OptimalCode]` class that takes doubles and `[FixedLength(n)] ReadOnlySpan<double>` inputs and
writes one `[FixedLength(n)] Span<double>` output is an **array kernel** (a finite element's residual, for example).
Attributes on it generate derivatives into `{Class}Kernels.g.cs`; each generated method allocates nothing and checks
span lengths on entry, and the scalar output above is unchanged.

| Attribute | Generates | How |
|---|---|---|
| `[Jacobian(nameof(u))]` | `{Kernel}Jacobian_u`: the dense ∂output/∂u, row-major | vector forward mode |
| `[Vjp(nameof(X), nameof(E))]` | `{Kernel}Vjp_X_E`: λᵀ∂output/∂X and λᵀ∂output/∂E | one reverse sweep |
| `[Hessian(nameof(F))]` | `{Kernel}Hessian_F`: the dense ∂²output/∂F² of a kernel with one output (an energy), exactly symmetric | forward over reverse |
| `[Hvp(nameof(F))]` | `{Kernel}Hvp_F`: (∂²output/∂F²) v, with v the `direction` input | one forward direction over reverse |

```csharp
[OptimalCode]
public static class Tet4Elastic
{
    [Jacobian(nameof(u))]
    [Vjp(nameof(X))]
    public static void Residual([FixedLength(12)] ReadOnlySpan<double> u, [FixedLength(12)] ReadOnlySpan<double> X,
        double E, double nu, [FixedLength(12)] Span<double> r) { /* … */ }
}
```

The generator (`AutoDiff.Kernels`) interprets the kernel at compile time: ints and bools are compile-time values, so
every loop is unrolled and every index is a constant, while doubles become nodes of a hash-consed SSA graph that folds
constants and shares common subexpressions. A kernel may use `stackalloc` and local arrays, collection expressions,
static readonly tables (`int[]`, `double[,]`, `ReadOnlySpan<int>` properties), `Slice`, ranges, `Length`, `Clear`,
`Fill` and `CopyTo`, `for` and `while` with `break` and `continue`, and calls to static helpers of `[OptimalCode]`
classes, which are inlined (spans pass by reference). The `System.Math` functions it may call are Abs, Atan, Atan2,
Cbrt, Clamp, Cos, Exp, Log, Max, Min, Pow (constant exponent), Sign, Sin, Sqrt, Tan and Tanh. `KernelMath.SmoothMin`,
`SmoothMax` and `SmoothAbs` are smooth stand-ins for Min, Max and Abs. Generated methods of more than `ChunkSize`
operations (500 by default) are split into chunks so the JIT optimises them.

**Structs.** A struct whose instance state is `double` fields, or fields of another such struct (a 3 × 3 tensor, say),
is flattened to its fields. It may be a local, a helper's parameter (by value or `in`) or return value, the result of a
conditional expression, and a field of another struct. It is built by a constructor, `new T()`, `default`, an object
initialiser or `with`, and read and written field by field (`p.Second.X += x`). Its constructors, instance methods,
get-only properties and operators are inlined; inside them `this` is the value, and a method that changes `this` writes
it back to the variable it was called on, as C# does. The struct must be non-generic, with source in the compilation,
and without auto-properties or a primary constructor, whose state the generator cannot see.

**Run-time conditions.** A comparison of doubles is decided at run time. In a conditional expression `c ? a : b` it is a
select. An `if` on one is **if-converted**: both branches are interpreted from the same state, and every variable,
struct field and array element (outputs included) that either branch assigns becomes a select of the two branches'
values. The generated code computes what C# computes. Within such a branch:
- `return`, `break` and `continue` are not allowed (put the rest of the method in an `else`);
- an int, bool or span may be assigned only if both branches leave it the same, because those are compile-time;
- an array element written in one branch must already hold a value, so the other branch has one to keep;
- `&&` and `||` of run-time comparisons are not allowed; nest the `if`s instead.

The derivative of a select, of Min, Max, Clamp and Abs, and of an if-converted branch is the derivative of the branch
the comparison takes, so it is the derivative of the function as written wherever the comparison is not at a tie. **On
the switching surface it is not defined**: there the generated derivative is the one-sided derivative of the branch
chosen (`if (f > 0)` takes the `else` branch at f = 0; Min and Max follow their first operand at a tie, Abs takes 0), and
where the branches do not meet in value the jump is not part of it. Use `KernelMath`'s smooth stand-ins where a kink
matters. A branch not taken may compute a NaN or an infinity (0/0 at zero deviatoric stress, say); it never reaches the
result or either derivative: forward mode selects tangents, and reverse mode never multiplies a select's zero adjoint by
the not-taken branch's partials.

**Implicit solves.** `KernelMath.ImplicitSolve(Residual, p, y, iterations)` solves a small nonlinear system
g(y, p) = 0 of up to 16 unknowns, from the guess in `y`, and leaves the solution there. `Residual` is a static method
`(ReadOnlySpan<double> y, ReadOnlySpan<double> p, Span<double> g)` named directly, not a lambda, so the generator can
inline it:

```csharp
Span<double> dGamma = stackalloc double[1];          // the guess, 0
KernelMath.ImplicitSolve(VoceResidual, p, dGamma, 12);
```

The generator builds g and its exact Jacobian ∂g/∂y once, unrolls `iterations` Newton steps, each solving the linear
system by Gaussian elimination with partial pivoting (the pivot choices are run-time selects), and then detaches the
iterate: no derivative flows through the iterations. One more Newton step with ∂g/∂y detached gives the value and the
derivative of the implicit-function theorem, dy/dp = −(∂g/∂y)⁻¹ ∂g/∂p, in forward and reverse mode alike; when the kernel
asks for a Hessian a further Newton step makes the solution exact to second order. The run-time method, which the
kernel's own code calls, takes the Jacobian by central differences, stops once a step is below 1e-13 relative, and
throws unless the last of its `iterations` steps is below 1e-8 relative. Choose `iterations` for convergence: the
generated code runs exactly that many steps, with no test.

Anything else is a compile error, never a silent mis-differentiation:

| Id | Raised when |
|---|---|
| OPTAD100 | A loop bound, index or length depends on a double; a run-time branch returns, breaks or continues, or leaves an int, bool or span different |
| OPTAD101 | The statement or expression is outside the kernel language, or a struct cannot be flattened |
| OPTAD102 | The kernel's signature is not as above |
| OPTAD103 | A call is not a supported Math function, span method or inlinable helper |
| OPTAD104 | A compile-time index or slice is out of range |
| OPTAD105 | An output element is never written, a value is read before it is written, an element is written in one branch of a run-time `if` only, or a residual leaves an equation unwritten |
| OPTAD106 | Unrolling exceeds 1,000,000 iterations, 4,000,000 graph nodes or 64 nested calls |
| OPTAD107 | A derivative attribute names no input, repeats another, sets `ChunkSize` below 1, or asks a Hessian of a kernel with more than one output |
| OPTAD108 | `Math.Pow`'s exponent is not a compile-time constant |
| OPTAD109 | The kernel writes to a static table |
| OPTAD110 | An implicit solve's residual is not a named static method of that signature, or it has more than 16 unknowns or fewer than one iteration |

## Code quality

- `Nullable` is enabled across every project.
- `TreatWarningsAsErrors = true` — including nullable warnings.
- Roslynator analyzers (`Analyzers`, `CodeAnalysis.Analyzers`, `Formatting.Analyzers`) run on every build.
- `AnalysisLevel` is `latest-Recommended`.
- Style & formatting are enforced by the repository `.editorconfig`.

## Contributing

1. Fork and create a feature branch.
2. Ensure `./utils/run-build.sh` and `./utils/run-tests.sh` both succeed.
3. Keep coverage from regressing — `./src/build.sh` will build the coverage report.
4. Match the existing code style (`.editorconfig` + Roslynator will tell you if you don't).
5. Open a pull request describing the change and its motivation.

## License

Source files are marked as MIT-licensed (see the file headers). A top-level `LICENSE` file will accompany a formal release.

Copyright © Small Trading Company Ltd (Destash.com).
