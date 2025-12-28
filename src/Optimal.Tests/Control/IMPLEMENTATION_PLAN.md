# Hermite-Simpson Collocation Control Optimization Implementation Plan

## Project Overview

Implement Hermite-Simpson collocation method for solving optimal control problems. This will be built within the `Optimal.Control` namespace in the `Optimal.Tests` project, leveraging existing nonlinear solvers (L-BFGS, Conjugate Gradient, Augmented Lagrangian) and AutoDiff capabilities.

## Implementation Status
- [x] Phase 1: Foundation & Problem Definition ✅ **COMPLETED**
- [ ] Phase 2: Hermite-Simpson Transcription
- [ ] Phase 3: Objective Function Integration
- [ ] Phase 4: NLP Integration with Existing Solvers
- [ ] Phase 5: Boundary Conditions & Path Constraints
- [ ] Phase 6: Mesh Refinement
- [ ] Phase 7: Classic Test Problems
- [ ] Phase 8: Warm Starting & Continuation
- [ ] Phase 9: Multi-Phase Problems
- [ ] Phase 10: Documentation & Examples

---

## Phase 1: Foundation & Problem Definition
**Status**: ✅ **COMPLETED** (2025-12-28)
**Goal**: Create core data structures for control problems and collocation.

### Checklist
- [x] Create `Control/` and `Control.Tests/` folders under `Optimal.Tests`
- [x] Create `ControlProblem.cs` - Define optimal control problem structure
- [x] Create `DynamicalSystem.cs` - System dynamics `ẋ = f(x, u, t)` (deferred to Phase 2)
- [x] Create `CollocationResult.cs` - Solution trajectory and diagnostics
- [x] Create `CollocationGrid.cs` - Time discretization and mesh management
- [x] Create basic test with simple 1D dynamics (integrator: `ẋ = u`)
- [x] Validate that AutoDiff can differentiate dynamics functions

### Key Design Decisions
- State vector: `x(t)` (continuous), Control vector: `u(t)` (piecewise)
- Time horizon: `[t0, tf]` with N collocation segments
- AutoDiff for all derivatives (dynamics, objective, constraints)
- Follow existing project patterns (builder API, record types, etc.)

### Implementation Notes
**Date Completed**: 2025-12-28

**Files Created**:
1. `Control/CollocationGrid.cs` - Uniform time grid with segments
2. `Control/CollocationResult.cs` - Immutable result record
3. `Control/ControlProblem.cs` - Builder API for problem specification
4. `Control.Tests/TestDynamics.cs` - AutoDiff test functions
5. `Control.Tests/CollocationGridTests.cs` - 7 passing tests
6. `Control.Tests/ControlProblemTests.cs` - 5 passing tests

**Test Results**: 12/12 tests passing (100%)

**AutoDiff Integration**:
- Successfully generated gradients for `SimpleIntegrator(x, u)`
- Successfully generated gradients for `DoubleIntegratorDx0(x0, x1, u)` and `DoubleIntegratorDx1(x0, x1, u)`
- **Key Learning**: Tuple returns not supported - use separate functions for multi-output dynamics
- Generated code pattern: `FunctionNameReverse(params...) => (value, gradients[])`

**Testing Framework**:
- Using MSTest (not Xunit): `[TestClass]`, `[TestMethod]`, `Assert.AreEqual`
- `OptimalCodeAttribute` available globally in `Optimal` namespace
- Static readonly arrays preferred over constant array allocations (CA1861 warning)

**Design Patterns**:
- Builder pattern with fluent API and method chaining
- Immutable record types for results
- Nullable reference types for optional properties
- Validation in setter methods

**Next Phase**: Ready for Phase 2 - Hermite-Simpson Transcription

---

## Phase 2: Hermite-Simpson Transcription
**Status**: Not Started
**Goal**: Implement collocation equations that convert ODE into NLP.

### Checklist
- [ ] Create `HermiteSimpsonTranscription.cs`
- [ ] Implement defect constraints at midpoints
- [ ] Create decision variable vector: `[x0, u0, x1, u1, ..., xN, uN]`
- [ ] Implement Hermite interpolation for states at midpoints
- [ ] Create `DefectConstraints.cs` - Collocation equality constraints
- [ ] Test on 1D integrator problem with analytical solution
- [ ] Verify transcription satisfies dynamics to tolerance

### Hermite-Simpson Formulation
For each segment `[t_k, t_{k+1}]`:

```
x_mid = (x_k + x_{k+1})/2 + h/8 * (f_k - f_{k+1})
u_mid = (u_k + u_{k+1})/2
Defect = x_{k+1} - x_k - h/6 * (f_k + 4*f_mid + f_{k+1}) = 0
```

Where:
- `h = t_{k+1} - t_k` (time step)
- `f_k = f(x_k, u_k, t_k)` (dynamics at node k)
- `f_mid = f(x_mid, u_mid, t_mid)` (dynamics at midpoint)

### Implementation Notes
[To be filled during Phase 2]

---

## Phase 3: Objective Function Integration
**Status**: Not Started
**Goal**: Add cost functional evaluation.

### Checklist
- [ ] Create `ObjectiveFunctional.cs` - Lagrange + Mayer terms
- [ ] Implement running cost integration: `∫ L(x, u, t) dt`
- [ ] Implement terminal cost: `Φ(x_f, t_f)`
- [ ] Use Simpson's rule for quadrature
- [ ] Test on minimum energy problem: `min ∫ u² dt`
- [ ] Test on time-optimal problem: `min t_f`

### Cost Structure
```
J = Φ(x(t_f), t_f) + ∫[t0, tf] L(x(t), u(t), t) dt
```

Simpson approximation:
```
J ≈ Φ(x_N, t_N) + Σ_k h/6 * (L_k + 4*L_mid + L_{k+1})
```

### Implementation Notes
[To be filled during Phase 3]

---

## Phase 4: NLP Integration with Existing Solvers
**Status**: Not Started
**Goal**: Connect collocation to nonlinear optimizers.

### Checklist
- [ ] Create `CollocationOptimizer.cs`
- [ ] Map collocation problem to `IOptimizer` interface
- [ ] Build augmented objective with defect constraint penalties
- [ ] Use `AugmentedLagrangianOptimizer` for equality constraints
- [ ] Test with L-BFGS as inner optimizer
- [ ] Solve double integrator: `ẍ = u`, min energy
- [ ] Validate solution satisfies dynamics and optimality conditions

### NLP Formulation
**Decision Vector:**
```
z = [x_0, u_0, x_1, u_1, ..., x_N, u_N]  // All states and controls
```

**Constraints:**
```
h_defect(z) = 0  // N defect constraints
h_boundary(z) = 0  // Initial/terminal conditions
g_path(z) ≤ 0  // Path constraints on x, u
```

### Implementation Notes
[To be filled during Phase 4]

---

## Phase 5: Boundary Conditions & Path Constraints
**Status**: Not Started
**Goal**: Add realistic problem constraints.

### Checklist
- [ ] Create `BoundaryCondition.cs` - Initial/final state constraints
- [ ] Add control bounds: `u_min ≤ u ≤ u_max`
- [ ] Add state bounds: `x_min ≤ x ≤ x_max`
- [ ] Test on pendulum swing-up with torque limits
- [ ] Test on cart-pole with position/velocity limits
- [ ] Verify constraint violation < tolerance

### Boundary Condition Types
- **Fixed initial state**: `x(t0) = x0`
- **Free final state**: no constraint on `x(tf)`
- **Target final state**: `x(tf) = xf`
- **Periodic**: `x(tf) = x(t0)`

### Implementation Notes
[To be filled during Phase 5]

---

## Phase 6: Mesh Refinement
**Status**: Not Started
**Goal**: Adaptive grid for accuracy vs. efficiency.

### Checklist
- [ ] Create `MeshRefinement.cs`
- [ ] Implement defect-based error estimation
- [ ] Add segments where defects are large
- [ ] Implement coarsening for uniform accuracy
- [ ] Test convergence with mesh refinement
- [ ] Compare coarse (N=10) vs fine (N=100) solutions

### Refinement Strategy
1. Solve with initial mesh (e.g., N=20)
2. Evaluate defect magnitudes
3. Subdivide segments with large defects
4. Re-solve on refined mesh
5. Repeat until error < tolerance

### Implementation Notes
[To be filled during Phase 6]

---

## Phase 7: Classic Test Problems
**Status**: Not Started
**Goal**: Validate against literature benchmarks.

### Checklist
- [ ] Implement Bryson-Denham problem (min ∫u² dt, state constraint)
- [ ] Implement Van der Pol oscillator
- [ ] Implement Goddard rocket problem
- [ ] Implement minimum-time double integrator
- [ ] Compare solutions to known analytical/numerical results
- [ ] Document convergence rates and accuracy

### Implementation Notes
[To be filled during Phase 7]

---

## Phase 8: Warm Starting & Continuation
**Status**: Not Started
**Goal**: Improve solver robustness.

### Checklist
- [ ] Implement linear interpolation for initial guess
- [ ] Add warm start from previous solution
- [ ] Implement parameter continuation (homotopy)
- [ ] Test difficult initialization scenarios
- [ ] Test parameter sweeps (e.g., varying target position)

### Implementation Notes
[To be filled during Phase 8]

---

## Phase 9: Multi-Phase Problems
**Status**: Not Started
**Goal**: Handle problems with distinct phases.

### Checklist
- [ ] Create `MultiPhaseControlProblem.cs`
- [ ] Implement phase linkage constraints
- [ ] Support variable phase durations
- [ ] Test on rocket ascent with staging
- [ ] Test on gait optimization with stance/swing phases

### Implementation Notes
[To be filled during Phase 9]

---

## Phase 10: Documentation & Examples
**Status**: Not Started
**Goal**: Make the library usable.

### Checklist
- [ ] Create `CONTROL_README.md` with API overview
- [ ] Add worked example: minimum-time car
- [ ] Add worked example: quadrotor trajectory
- [ ] Add worked example: robot arm motion planning
- [ ] Document convergence tips and troubleshooting

### Implementation Notes
[To be filled during Phase 10]

---

## Example Target API

```csharp
// Define dynamics: double integrator ẍ = u
[OptimalCode]
public static class IntegratorDynamics
{
    public static double[] Dynamics(double[] x, double[] u, double t)
    {
        // x = [position, velocity]
        // u = [acceleration]
        return new[] { x[1], u[0] };
    }
}

// Define objective: minimize control effort
[OptimalCode]
public static class QuadraticCost
{
    public static double RunningCost(double[] x, double[] u, double t)
    {
        return 0.5 * u[0] * u[0];
    }
}

// Solve optimal control problem
var problem = new ControlProblem()
    .WithDynamics((x, u, t) => IntegratorDynamicsGradients.DynamicsReverse(x, u, t))
    .WithRunningCost((x, u, t) => QuadraticCostGradients.RunningCostReverse(x, u, t))
    .WithStateSize(2)
    .WithControlSize(1)
    .WithTimeHorizon(0.0, 10.0)
    .WithInitialCondition(new[] { 0.0, 0.0 })
    .WithFinalCondition(new[] { 1.0, 0.0 })
    .WithControlBounds(new[] { -1.0 }, new[] { 1.0 });

var solver = new HermiteSimpsonSolver()
    .WithGrid(20)  // 20 collocation segments
    .WithTolerance(1e-6)
    .WithInnerOptimizer(new LBFGSOptimizer());

var result = solver.Solve(problem);

// Access solution
double[] times = result.Times;
double[][] states = result.States;  // [N+1][state_dim]
double[][] controls = result.Controls;  // [N+1][control_dim]
```

---

## Integration Points with Existing Code

### Nonlinear Solvers
- Use `LBFGSOptimizer` for unconstrained NLP subproblems
- Use `ConjugateGradientOptimizer` as alternative solver
- Use `AugmentedLagrangianOptimizer` for defect constraints

### AutoDiff Integration
- Mark dynamics functions with `[OptimalCode]` attribute
- Mark objective functions with `[OptimalCode]` attribute
- Generated gradients: `FunctionNameReverse(...)` returns `(value, gradients[])`
- Pattern: inline expressions without local variables for reverse-mode AD

### Constrained Optimization
- Reuse `IConstraint`, `EqualityConstraint`, `InequalityConstraint`
- Reuse `BoxConstraints` for control/state bounds
- Leverage existing augmented Lagrangian infrastructure

### Convergence Monitoring
- Reuse `ConvergenceMonitor` for solver diagnostics
- Reuse `StoppingReason` enum
- Follow existing result record pattern

### Testing Infrastructure
- Follow naming: `CanSolveDoubleIntegrator`, NOT `Can_Solve_Double_Integrator`
- Use `[OptimalCode]` test functions with inline expressions
- Build and run tests with: `cd src && ./build.sh`

---

## Key References

1. **Hermite-Simpson Method**: Hargraves & Paris (1987), "Direct Trajectory Optimization Using Nonlinear Programming and Collocation"
2. **GPOPS-II**: Patterson & Rao (2014) - MATLAB implementation reference
3. **Betts**: "Practical Methods for Optimal Control Using Nonlinear Programming" (textbook)
4. **NLOPT Documentation**: https://nlopt.readthedocs.io/en/latest/
5. **Existing NonLinear Implementation**: `/src/Optimal.Tests/NonLinear/PROGRESS.md`

---

## Design Philosophy

### Minimal Changes
- Reuse existing solver infrastructure wherever possible
- Follow established patterns from `NonLinear/` namespace
- Keep implementation surgical and focused

### Builder Pattern
- Use fluent API for problem specification
- Method chaining for configuration
- Immutable problem/result records

### AutoDiff First
- All derivatives via AutoDiff (no manual Jacobians)
- Mark functions with `[OptimalCode]`
- Use inline expressions for reverse-mode compatibility

### Testing Strategy
- Start simple (1D integrator) and build complexity
- Compare to analytical solutions where available
- Validate dynamics satisfaction and optimality conditions
- Test convergence rates and mesh refinement

---

## Implementation Timeline

This is an iterative plan. Complete each phase fully before moving to the next:

1. **Week 1**: Phases 1-2 (Foundation + Transcription)
2. **Week 2**: Phases 3-4 (Objective + NLP Integration)
3. **Week 3**: Phases 5-6 (Constraints + Mesh Refinement)
4. **Week 4**: Phases 7-8 (Test Problems + Warm Start)
5. **Future**: Phases 9-10 (Multi-Phase + Documentation)

Start with Phase 1 to establish solid foundations!
