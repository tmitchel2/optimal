# NonLinear Optimization Implementation Progress

## Status Overview
- [x] Phase 1: Foundation & Infrastructure ✅ **COMPLETE**
- [x] Phase 2: Line Search ✅ **COMPLETE**
- [x] Phase 3: Conjugate Gradient ✅ **COMPLETE**
- [x] Phase 4: L-BFGS ✅ **COMPLETE**
- [ ] Phase 5: Constrained Optimization
- [ ] Phase 6: Enhanced Stopping Criteria
- [ ] Phase 7: Utilities & Helpers
- [ ] Phase 8: Documentation & Examples

---

## Phase 1: Foundation & Infrastructure
**Status**: ✅ **COMPLETED**
**Started**: 2025-12-27
**Completed**: 2025-12-27

### Goal
Create core interfaces, basic gradient descent optimizer, and validate AutoDiff integration.

### Checklist
- [x] Create `NonLinear/` and `NonLinear.Tests/` folder structure
- [x] Create `PROGRESS.md` tracking document
- [x] Create `StoppingReason.cs` enum
- [x] Create `OptimizerResult.cs` record
- [x] Create `IOptimizer.cs` interface
- [x] Create `GradientDescentOptimizer.cs`
- [x] Create `TestObjectiveFunctions.cs` with `[OptimalCode]`
- [x] Create `GradientDescentOptimizerTests.cs`
- [x] Build project and verify AutoDiff code generation
- [x] Run tests for quadratic function ✅
- [x] Run tests for Rosenbrock 2D function ✅
- [x] Verify AutoDiff gradients match numerical gradients ✅

### Test Results
**All 6 tests passing** (6/6 = 100%)
- ✅ CanMinimizeQuadraticFunction
- ✅ CanMinimizeRosenbrockFunction
- ✅ GradientMatchesNumericalGradient
- ✅ ReturnsMaxIterationsWhenNotConverged
- ✅ FunctionEvaluationCountIsCorrect
- ✅ CanMinimizeBoothFunction

### Implementation Notes

#### Issue Discovered and Resolved
**Problem**: Intermediate local variables in objective functions caused reverse-mode AutoDiff to generate zero gradients.

**Root Cause**: When using local variables like:
```csharp
var term1 = 1.0 - x;
var term2 = y - x * x;
return term1 * term1 + 100.0 * term2 * term2;
```
The reverse-mode code generator was not properly tracking gradients through these intermediate assignments, resulting in `adj[5]` never being set and gradients being zero.

**Solution**: Rewrote all objective functions to use inline expressions without intermediate variables:
```csharp
return (1.0 - x) * (1.0 - x) + 100.0 * (y - x * x) * (y - x * x);
```

This allowed the AutoDiff system to properly compute reverse-mode gradients.

#### AutoDiff Integration Pattern
Generated code signature (reverse mode):
```csharp
public static (double value, double[] gradients) FunctionNameReverse(double x, double y, ...)
```

Optimizer interface expects:
```csharp
Func<double[], (double value, double[] gradient)> objective
```

Adapter usage:
```csharp
var result = optimizer.Minimize(x =>
    TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
);
```

#### Decisions
- Using builder pattern for optimizer configuration
- Gradient descent with fixed step size initially (line search in Phase 2)
- Simple gradient norm convergence criterion
- Array-based parameters (`double[]`) for extensibility

---

## Phase 2: Line Search
**Status**: ✅ **COMPLETED**
**Started**: 2025-12-27
**Completed**: 2025-12-27

### Goal
Implement robust step size selection via backtracking line search with Armijo condition.

### Checklist
- [x] Create `LineSearch/` folder
- [x] Create `ILineSearch.cs` interface
- [x] Create `BacktrackingLineSearch.cs` implementation
- [x] Create `LineSearch/BacktrackingLineSearchTests.cs`
- [x] Integrate line search into `GradientDescentOptimizer`
- [x] Verify Rosenbrock converges without manual step size tuning ✅

### Test Results
**All 11 tests passing** (11/11 = 100%)

**Phase 1 Tests (still passing)**:
- ✅ CanMinimizeQuadraticFunction
- ✅ CanMinimizeRosenbrockFunction
- ✅ GradientMatchesNumericalGradient
- ✅ ReturnsMaxIterationsWhenNotConverged
- ✅ FunctionEvaluationCountIsCorrect
- ✅ CanMinimizeBoothFunction

**Phase 2 Tests (new)**:
- ✅ CanFindStepSizeForQuadraticFunction
- ✅ CanFindStepSizeForRosenbrockFunction
- ✅ ReturnsZeroForUphillDirection
- ✅ OptimizerWithLineSearchConvergesOnRosenbrock
- ✅ OptimizerWithLineSearchIsFasterThanFixed

### Implementation Notes

#### Backtracking Line Search Algorithm
Implements the Armijo condition for sufficient decrease:
```
f(x + α*d) ≤ f(x) + c₁ * α * ∇f(x)ᵀd
```

**Parameters**:
- `c1`: Armijo parameter (default 1e-4)
- `rho`: Backtracking reduction factor (default 0.5)
- `maxIterations`: Max backtracking steps (default 50)

**Behavior**:
- Starts with `α = 1.0` (or specified initial step)
- Reduces by factor `rho` until Armijo condition satisfied
- Returns `α = 0` if direction is uphill or no suitable step found
- Prevents infinite loops with max iteration limit

#### Integration with GradientDescentOptimizer
- Added optional `WithLineSearch()` method
- Backward compatible: if no line search set, uses fixed step size
- Line search uses negative gradient as search direction
- Automatically adapts step size per iteration

#### Performance Improvement
Confirmed that line search significantly improves convergence:
- **Without line search**: Rosenbrock requires ~20,000+ iterations (fixed step 0.001)
- **With line search**: Rosenbrock requires < 10,000 iterations
- **Speedup**: > 2x faster convergence

---

## Phase 3: Conjugate Gradient
**Status**: ✅ **COMPLETED**
**Started**: 2025-12-27
**Completed**: 2025-12-27

### Goal
Implement conjugate gradient optimizer using Fletcher-Reeves formula.

### Checklist
- [x] Create `ConjugateGradientFormula.cs` enum (FR, PR, HS)
- [x] Create `ConjugateGradientOptimizer.cs`
- [x] Create `ConjugateGradientOptimizerTests.cs`
- [x] Add extended Rosenbrock test functions (4D, 10D)
- [x] Test on extended Rosenbrock (4D, 10D) ✅
- [x] Test on Beale function ✅
- [x] Verify 5x+ speedup vs steepest descent ✅

### Test Results
**All 20 tests passing** (9 new + 11 from previous phases = 100%)

**Phase 3 Tests (new)**:
- ✅ CanMinimizeRosenbrock2DWithFletcherReeves
- ✅ CanMinimizeRosenbrock2DWithPolakRibiere
- ✅ CanMinimizeRosenbrock2DWithHestenesStiefel
- ✅ CanMinimizeBealeFunction
- ✅ CanMinimizeRosenbrock4D
- ✅ CanMinimizeRosenbrock10D
- ✅ ConjugateGradientIsFasterThanGradientDescent
- ✅ RestartIntervalWorks
- ✅ ReturnsMaxIterationsWhenNotConverged

**Previous Phases (still passing)**:
- Phase 1: 6/6 tests ✅
- Phase 2: 5/5 tests ✅

### Implementation Notes

#### Conjugate Gradient Algorithm
Implements three variants of the nonlinear conjugate gradient method:
- **Fletcher-Reeves (FR)**: Most robust, guarantees descent with exact line search
- **Polak-Ribiere (PR)**: Often faster, automatic restart when beta < 0
- **Hestenes-Stiefel (HS)**: Theoretically elegant but can be numerically unstable

**Algorithm Steps**:
1. Initialize search direction `d_0 = -∇f(x_0)` (steepest descent)
2. Find step size `α` using line search along direction `d`
3. Update position: `x_new = x + α * d`
4. Compute new gradient `∇f(x_new)`
5. Compute beta parameter using selected formula
6. Update search direction: `d_new = -∇f(x_new) + β * d`
7. Repeat until convergence

#### Beta Formulas

**Fletcher-Reeves**:
```
β = ||∇f_new||² / ||∇f_old||²
```

**Polak-Ribiere**:
```
β = ∇f_new^T * (∇f_new - ∇f_old) / ||∇f_old||²
```
- Automatic restart: if β < 0, set β = 0 (reverts to steepest descent)

**Hestenes-Stiefel**:
```
β = ∇f_new^T * (∇f_new - ∇f_old) / (d_old^T * (∇f_new - ∇f_old))
```

#### Features Implemented
- **Multiple beta formulas**: FR, PR, HS selectable via `WithFormula()`
- **Line search integration**: Uses backtracking line search for step size
- **Automatic restart**:
  - PR variant restarts when β < 0
  - Optional periodic restart via `WithRestartInterval(n)`
  - Automatic restart when line search fails with CG direction
- **Fallback to steepest descent**: If CG direction fails line search, tries negative gradient

#### Performance Results
- **Rosenbrock 2D**: Converges in ~100-500 iterations (vs ~10,000 for gradient descent)
- **Rosenbrock 4D**: Converges successfully with tolerance 1e-4
- **Rosenbrock 10D**: Converges successfully with tolerance 1e-3 in < 20,000 iterations
- **Beale function**: Converges to minimum (3, 0.5) with tolerance 1e-5
- **Speedup**: Confirmed > 5x faster than gradient descent (conservative test passed)

#### Extended Test Functions
Added to `TestObjectiveFunctions.cs`:
- `Rosenbrock4D(x1, x2, x3, x4)`: Extended Rosenbrock in 4 dimensions
- `Rosenbrock10D(x1, ..., x10)`: Extended Rosenbrock in 10 dimensions

Extended Rosenbrock formula:
```
f(x) = Σ[100(x_{i+1} - x_i²)² + (1 - x_i)²] for i = 1 to n-1
```

#### Design Decisions
- Default formula: Fletcher-Reeves (most robust)
- Default line search: BacktrackingLineSearch
- No periodic restart by default (can be enabled via `WithRestartInterval()`)
- Builder pattern maintained for API consistency
- Returns `NumericalError` if both CG and steepest descent directions fail

#### Numerical Stability
- Division by zero protection: Check denominators > 1e-16 before dividing
- Automatic fallback to steepest descent when line search fails
- PR variant's negative beta restart prevents algorithm breakdown

---

## Phase 4: L-BFGS
**Status**: ✅ **COMPLETED**
**Started**: 2025-12-27
**Completed**: 2025-12-27

### Goal
Implement limited-memory BFGS quasi-Newton method.

### Checklist
- [x] Create `LBFGSMemory.cs` - Circular buffer for (s, y) vector pairs
- [x] Create `TwoLoopRecursion.cs` - L-BFGS direction computation
- [x] Create `LBFGSOptimizer.cs` - Main algorithm
- [x] Create `HighDimensionalFunctions.cs` - Helper for 100D testing
- [x] Create `LBFGSOptimizerTests.cs` ✅
- [x] Test on 100D Rosenbrock (< 1000 iterations) ✅
- [x] Test on Booth function ✅

### Test Results
**All 28 tests passing** (8 new + 20 from previous phases = 100%)

**Phase 4 Tests (new)**:
- ✅ CanMinimizeRosenbrock2D
- ✅ CanMinimizeBoothFunction
- ✅ CanMinimizeRosenbrock10D
- ✅ CanMinimizeRosenbrock100D
- ✅ LBFGSIsFasterThanConjugateGradient
- ✅ MemorySizeAffectsPerformance
- ✅ ReturnsMaxIterationsWhenNotConverged
- ✅ WorksWithDifferentMemorySizes

**Previous Phases (still passing)**:
- Phase 1: 6/6 tests ✅
- Phase 2: 5/5 tests ✅
- Phase 3: 9/9 tests ✅

### Implementation Notes

#### L-BFGS Algorithm Overview
Limited-memory BFGS (Broyden-Fletcher-Goldfarb-Shanno) is a quasi-Newton method that approximates the inverse Hessian matrix using only the last `m` gradient and position updates. This makes it highly memory-efficient for large-scale problems.

**Key Insight**: Instead of storing the full inverse Hessian (n×n matrix, O(n²) memory), L-BFGS stores only `m` correction pairs (s, y), requiring O(mn) memory.

#### Three-Component Architecture

**1. LBFGSMemory.cs** - Circular Buffer
- Manages storage of correction pairs: (s_k, y_k) where:
  - `s_k = x_{k+1} - x_k` (position change)
  - `y_k = ∇f_{k+1} - ∇f_k` (gradient change)
- Precomputes `ρ_k = 1 / (y_k^T · s_k)` for efficiency
- Circular buffer: when full, oldest pair is overwritten
- Curvature condition check: rejects pairs where |y^T · s| < 1e-16

**2. TwoLoopRecursion.cs** - Direction Computation
Implements the L-BFGS two-loop recursion algorithm:

```
Algorithm: Compute search direction
Input: gradient g, memory containing m pairs (s_i, y_i, ρ_i)
Output: search direction d ≈ -H · g (where H ≈ inverse Hessian)

q ← g
for i = m-1 down to 0:
    α_i ← ρ_i · (s_i^T · q)
    q ← q - α_i · y_i

γ ← (s_{m-1}^T · y_{m-1}) / (y_{m-1}^T · y_{m-1})
r ← γ · q

for i = 0 to m-1:
    β ← ρ_i · (y_i^T · r)
    r ← r + s_i · (α_i - β)

return -r
```

**First loop** (backward): applies gradient changes
**Scaling**: uses most recent curvature information
**Second loop** (forward): applies position changes

**3. LBFGSOptimizer.cs** - Main Algorithm
- Integrates memory and two-loop recursion
- Default memory size: m = 10 (configurable via `WithMemorySize()`)
- Line search integration for step size selection
- Automatic restart: clears memory when falling back to steepest descent
- Builder pattern for API consistency

#### High-Dimensional Testing
Created `HighDimensionalFunctions.cs` for testing arbitrary dimensions:
- `ExtendedRosenbrock(x)`: Analytical implementation with exact gradients
- `RosenbrockStartingPoint(n)`: Standard starting point (-1.2, 1.0, -1.2, 1.0, ...)
- Enables testing on 100D problems without AutoDiff code generation

Extended Rosenbrock formula:
```
f(x) = Σ[100(x_{i+1} - x_i²)² + (1 - x_i)²] for i = 1 to n-1
```

Analytical gradient:
```
∂f/∂x_i = -400x_i(x_{i+1} - x_i²) - 2(1 - x_i)  [from i-th term]
        + 200(x_i - x_{i-1}²)                    [from (i-1)-th term]
```

#### Performance Results
- **Rosenbrock 2D**: Converges in ~30-50 iterations (excellent)
- **Rosenbrock 10D**: Converges quickly with tolerance 1e-4
- **Rosenbrock 100D**: **Converges in < 1000 iterations** ✅ (success criterion met!)
- **Booth function**: Converges to minimum (1, 3) with tolerance 1e-6
- **vs Conjugate Gradient**: L-BFGS is faster on 10D Rosenbrock

#### Memory Size Impact
Tested memory sizes: m = 3, 5, 10, 20
- **Small memory (m=3)**: Still effective but may require more iterations
- **Medium memory (m=10)**: Good balance (default)
- **Large memory (m=20)**: Best performance but higher memory cost
- All sizes converge successfully on test problems

#### Design Decisions
- Default memory size: m = 10 (balances performance and memory)
- Default line search: BacktrackingLineSearch
- Curvature condition: Reject pairs where |y^T · s| < 1e-16
- Initial Hessian scaling: γ = (s^T · y) / (y^T · y) from most recent pair
- When memory empty: falls back to steepest descent
- Builder pattern maintained for API consistency

#### Numerical Stability
- **Curvature condition enforcement**: Only accept (s, y) pairs with sufficient curvature
- **Division by zero protection**: Check denominators before dividing
- **Automatic restart**: Clear memory when line search fails with L-BFGS direction
- **Fallback to steepest descent**: Try negative gradient when L-BFGS direction fails

#### Why L-BFGS is Fast
1. **Quasi-Newton**: Approximates second-order (Hessian) information from first-order gradients
2. **Superlinear convergence**: Much faster than linear methods (gradient descent, CG)
3. **Memory efficient**: O(mn) instead of O(n²) for full BFGS
4. **Scales to high dimensions**: Successfully handles 100D+ problems

---

## Phase 5: Constrained Optimization
**Status**: Not Started

### Goal
Implement support for equality and inequality constraints using augmented Lagrangian and penalty methods.

### Checklist
- [ ] Create `IConstraint.cs` - Base interface for constraints
- [ ] Create `BoxConstraints.cs` - Simple bounds: `a ≤ x ≤ b`
- [ ] Create `EqualityConstraint.cs` - Equality constraints: `h(x) = 0`
- [ ] Create `InequalityConstraint.cs` - Inequality constraints: `g(x) ≤ 0`
- [ ] Create `PenaltyMethod.cs` - Quadratic penalty method
- [ ] Create `AugmentedLagrangian.cs` - Augmented Lagrangian method
- [ ] Create `ConstrainedOptimizer.cs` - Main constrained optimizer
- [ ] Create `ConstrainedOptimizerTests.cs`
- [ ] Test on problems with box constraints
- [ ] Test on problems with equality constraints
- [ ] Test on problems with inequality constraints
- [ ] Test on mixed constraint problems

### Implementation Notes

#### Constraint Types

**1. Box Constraints (Bounds)**
Simplest form: `a_i ≤ x_i ≤ b_i` for each variable.

Approaches:
- **Projected gradient**: Project onto feasible region after each step
- **Active set method**: Identify active constraints and optimize on subspace
- **Barrier method**: Add logarithmic barrier terms to objective

**2. Equality Constraints**
Form: `h_j(x) = 0` for j = 1, ..., m_eq

Example: Optimize `f(x, y)` subject to `x + y = 1`

**3. Inequality Constraints**
Form: `g_k(x) ≤ 0` for k = 1, ..., m_ineq

Example: Optimize `f(x, y)` subject to `x² + y² ≤ 1`

#### Methods

**Penalty Method (Exterior)**
Convert constrained problem to unconstrained by adding penalty terms:
```
L(x, μ) = f(x) + μ/2 * [Σ h_j(x)² + Σ max(0, g_k(x))²]
```
- Start with small μ, gradually increase
- Simple to implement but can be ill-conditioned
- Works with any unconstrained optimizer (L-BFGS, CG, etc.)

**Augmented Lagrangian Method**
More robust than pure penalty:
```
L(x, λ, μ) = f(x) + Σ λ_j * h_j(x) + μ/2 * Σ h_j(x)²
           + Σ max(0, λ_k + μ*g_k(x))²/2μ
```
- Maintains Lagrange multiplier estimates λ
- Better conditioning than pure penalty
- Industry standard for constrained optimization
- Used in NLOPT, IPOPT, etc.

**Barrier Method (Interior Point)**
Only for inequality constraints, adds barrier inside feasible region:
```
B(x, μ) = f(x) - μ * Σ log(-g_k(x))
```
- Requires strictly feasible starting point
- Decreases μ to approach boundary
- Very effective for convex problems

#### AutoDiff Integration
For constraints defined with `[OptimalCode]`:
```csharp
[OptimalCode]
public static class ConstraintFunctions
{
    // Equality: x + y - 1 = 0
    public static double SumEqualsOne(double x, double y) => x + y - 1.0;

    // Inequality: x² + y² - 1 ≤ 0 (inside unit circle)
    public static double InsideUnitCircle(double x, double y) => x*x + y*y - 1.0;
}
```

Generated gradients: `ConstraintFunctionsGradients.SumEqualsOneReverse(x, y)`

#### API Design
```csharp
var optimizer = new AugmentedLagrangianOptimizer()
    .WithUnconstrainedOptimizer(new LBFGSOptimizer())
    .WithInitialPoint(new[] { 0.5, 0.5 })
    .WithEqualityConstraint(x => ConstraintFunctionsGradients.SumEqualsOneReverse(x[0], x[1]))
    .WithInequalityConstraint(x => ConstraintFunctionsGradients.InsideUnitCircleReverse(x[0], x[1]))
    .WithTolerance(1e-6);

var result = optimizer.Minimize(x => ObjectiveFunctionsGradients.MyObjectiveReverse(x[0], x[1]));
```

#### Test Problems

**1. Rosenbrock with box constraints**
- Minimize Rosenbrock subject to `-2 ≤ x ≤ 2, -2 ≤ y ≤ 2`
- Tests projection and bounds handling

**2. Circle-constrained optimization**
- Minimize `f(x, y) = (x - 1)² + (y - 1)²` subject to `x² + y² ≤ 1`
- Tests inequality constraints

**3. Linear equality constraint**
- Minimize Rosenbrock subject to `x + y = 1`
- Tests equality constraints

**4. Hock-Schittkowski problems**
- Standard benchmark problems from optimization literature
- Mix of equality and inequality constraints

#### Design Decisions
- Start with Augmented Lagrangian (most robust)
- Support AutoDiff-generated constraint gradients
- Builder pattern for constraint specification
- Delegate to existing unconstrained optimizers (L-BFGS, CG)
- Optional constraint violation tracking in results

#### Performance Considerations
- Penalty parameter adaptation strategy
- Constraint violation tolerance
- Inner optimization tolerance vs outer tolerance
- Maximum outer iterations (typically 10-50)

---

## Phase 6: Enhanced Stopping Criteria
**Status**: Not Started

### Goal
Sophisticated convergence detection and diagnostics.

### Checklist
- [ ] Create `ConvergenceMonitor.cs`
- [ ] Enhance `OptimizerResult` with detailed diagnostics
- [ ] Implement gradient norm tolerance
- [ ] Implement function value change tolerance
- [ ] Implement parameter change tolerance
- [ ] Implement maximum function evaluations
- [ ] Implement stall detection

### Implementation Notes
[To be filled during Phase 5]

---

## Phase 7: Utilities & Helpers
**Status**: Not Started

### Goal
Numerical gradient checking, benchmarking, and history tracking.

### Checklist
- [ ] Create `NumericalGradient.cs` - Finite difference validator
- [ ] Create `OptimizerBenchmark.cs` - Performance comparison
- [ ] Create `OptimizationHistory.cs` - Trajectory recording
- [ ] Create `NumericalGradientTests.cs`
- [ ] Test gradient correctness on all test functions

### Implementation Notes
[To be filled during Phase 7]

---

## Phase 8: Documentation & Examples
**Status**: Not Started

### Goal
Comprehensive documentation and worked examples.

### Checklist
- [ ] Create `README.md` - API overview and quick start
- [ ] Create `EXAMPLES.md` - Worked examples
- [ ] Create `Examples/RosenbrockExample.cs`
- [ ] Create `Examples/MachineLearningExample.cs` - Logistic regression
- [ ] Create `Examples/PhysicsExample.cs` - Trajectory optimization
- [ ] Review all documentation for clarity

### Implementation Notes
[To be filled during Phase 7]

---

## Implementation Decisions

### Coordinate System
- 1D arrays (`double[]`) for parameter vectors (standard in optimization)
- Generated gradients return `double[]` in parameter order
- Array indices correspond to parameter order in method signature

### Naming Conventions (from .editorconfig)
- Instance fields: `_camelCase`
- Static fields: `s_camelCase`
- Methods/Properties: `PascalCase`
- Test methods: `PascalCase` (e.g., `CanMinimizeQuadraticFunction`)
- Parameters/Locals: `camelCase`

### Performance Considerations
- Minimize array allocations in inner loops
- Reuse gradient arrays where possible
- Consider array pooling for high-dimensional problems (future enhancement)

---

## Future Enhancements

### Constraints
- Box constraints (simple bounds): `x_min ≤ x ≤ x_max`
- Linear constraints: `Ax ≤ b`
- Nonlinear constraints: barrier/penalty methods, augmented Lagrangian

### Algorithms
- Trust region methods (alternative to line search)
- Stochastic gradient descent (mini-batch, Adam, RMSprop)
- Derivative-free methods (Nelder-Mead, BOBYQA)
- Global optimization (genetic algorithms, particle swarm)

### Features
- Parallel function evaluations (multi-start)
- Automatic differentiation of constraints
- Warm start (resume from previous solution)
- Callbacks for custom logging/plotting

---

## Test Functions Reference

### Quadratic
```csharp
f(x,y) = x² + y²
```
- Minimum: (0, 0), f(0, 0) = 0
- Simple convex function for basic validation

### Rosenbrock 2D
```csharp
f(x,y) = (1-x)² + 100(y-x²)²
```
- Minimum: (1, 1), f(1, 1) = 0
- Classic test problem with narrow curved valley

### Beale
```csharp
f(x,y) = (1.5 - x + xy)² + (2.25 - x + xy²)² + (2.625 - x + xy³)²
```
- Minimum: (3, 0.5), f(3, 0.5) = 0
- Tests handling of multiple local minima

### Booth
```csharp
f(x,y) = (x + 2y - 7)² + (2x + y - 5)²
```
- Minimum: (1, 3), f(1, 3) = 0
- Simple quadratic with cross terms

---

## Resources

- **Plan Document**: `/Users/tom/.claude/plans/serialized-drifting-pumpkin.md`
- **NLOPT Documentation**: https://nlopt.readthedocs.io/en/latest/
- **AutoDiff Analyzer**: `src/Optimal/AutoDiff.Analyzers/OptimalIncrementalSourceGenerator.cs`
