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
