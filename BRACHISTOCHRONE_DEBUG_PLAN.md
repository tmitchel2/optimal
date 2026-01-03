# Brachistochrone Free Final Time - Debugging Plan

**Problem**: The BrachistochroneProblemSolver converges with fixed final time but fails to converge when formulated with free final time using time-scaling transformation.

**Current Formulation**:
- Time-scaling: τ ∈ [0,1], t = T_f·τ where T_f is the free final time
- State: [x, y, v, T_f] - T_f treated as 4th state variable
- Dynamics: dx/dτ = T_f · (dx/dt), dT_f/dτ = 0
- Cost: L(x,u,τ) = T_f (integrated over τ ∈ [0,1] gives total time)

---

## Phase 1: Verify Gradient Computations ✅ COMPLETED

### 1.1 Dynamics Gradient Verification ✅ COMPLETED
**File**: `BrachistochroneProblemSolver.cs:69-115`

**Issue**: The gradients ∂(dx/dτ)/∂T_f = dx/dt (physical rate) need verification.

**Actions**:
- [x] Add diagnostic logging to print dynamics gradients at initial point
- [x] Verify chain rule application: ∂(T_f·f)/∂x = T_f·(∂f/∂x), ∂(T_f·f)/∂T_f = f
- [x] Compare analytical gradients with numerical finite-difference approximation
- [x] Check for NaN or Inf values in gradient computation
- [x] Verify gradient array indexing matches expected layout:
  - gradients[0]: state gradients (4x4 matrix flattened)
  - gradients[1]: control gradients (4x1 vector)

**Result**: ✅ ALL TESTS PASSED
- Created comprehensive test suite in `BrachistochroneDynamicsTests.cs`
- All time-scaled dynamics gradients verified numerically (within 1e-5 tolerance)
- Running cost gradient ∂L/∂T_f = 1.0 verified
- Integrated cost equals T_f verified
- **Conclusion**: Gradient computations are mathematically correct - not the source of convergence issues

---

## Phase 2: Cost Function Analysis ⬜

### 2.1 Cost Gradient Structure ⬜
**File**: `BrachistochroneProblemSolver.cs:116-132`

**Issue**: Running cost L = T_f has constant gradient ∂L/∂T_f = 1.0, which may cause poor conditioning.

**Actions**:
- [ ] Log cost and gradient values at each iteration
- [ ] Check if cost is actually being minimized (should decrease)
- [ ] Verify gradient array structure matches solver expectations:
  - Expected: [∂L/∂x, ∂L/∂y, ∂L/∂v, ∂L/∂T_f, ∂L/∂θ, ∂L/∂τ]
  - Current: [0, 0, 0, 1, 0, 0]
- [ ] Compare integrated cost ∫T_f dτ with final T_f value (should equal)

### 2.2 Cost Scaling Investigation ⬜

**Issue**: The cost magnitude might be poorly scaled relative to state magnitudes.

**Actions**:
- [ ] Try adding a scaling factor to the cost: L = α·T_f (test α = 0.1, 1.0, 10.0)
- [ ] Consider alternative cost formulation: terminal cost Φ(x) = T_f instead of running cost
- [ ] Test hybrid formulation: small running cost + terminal cost

**Expected Output**: Identify if scaling affects convergence behavior

---

## Phase 3: Initial Guess Quality ⬜

### 3.1 T_f Initial Guess Sensitivity ⬜
**File**: `BrachistochroneProblemSolver.cs:47`

**Issue**: Initial guess tfGuess = 1.5 may be far from optimal, causing poor basin of attraction.

**Actions**:
- [ ] Compute analytical/approximate optimal time from Brachistochrone physics
  - For cycloid: T_opt ≈ π√(R/g) where R is radius
  - For given geometry, estimate expected time
- [ ] Test multiple initial guesses: [0.5, 1.0, 1.5, 2.0, 2.5]
- [ ] Check if solver converges from any of these guesses
- [ ] If only some converge, identify the basin of attraction

### 3.2 State Trajectory Initial Guess ⬜

**Issue**: The automatic initial guess may not account for T_f being a state variable.

**Actions**:
- [ ] Review `HermiteSimpsonSolver.cs:286-319` initial guess creation
- [ ] Verify that T_f component is properly initialized in all collocation points
- [ ] Check if T_f varies across trajectory in initial guess (should be constant)
- [ ] Create custom physics-based initial guess for full state trajectory

**Expected Output**: Initial guess should have T_f constant across all time points

---

## Phase 4: Bounds and Constraints ⬜

### 4.1 State Bounds Investigation ⬜
**File**: `BrachistochroneProblemSolver.cs:66-68`

**Issue**: T_f bounds [0.1, 5.0] may be too restrictive or cause optimizer issues.

**Actions**:
- [ ] Test with relaxed bounds: [0.01, 10.0]
- [ ] Test with tighter bounds around expected optimal: [1.0, 3.0]
- [ ] Check if optimizer is hitting the bounds (add logging)
- [ ] Verify bound violations don't cause gradient issues

### 4.2 Final Condition Constraints ⬜
**File**: `BrachistochroneProblemSolver.cs:64`

**Issue**: T_f has free final condition (double.NaN), but dT_f/dτ = 0 means it should be constant.

**Actions**:
- [ ] Verify that constant T_f is enforced by dynamics (dT_f/dτ = 0)
- [ ] Check defect constraints for T_f component
- [ ] Test alternative: explicitly constrain x_final[3] = x_initial[3]
- [ ] Verify boundary condition constraint implementation in solver

**Expected Output**: T_f should remain constant throughout trajectory within tolerance

---

## Phase 5: Optimizer Configuration ⬜

### 5.1 Tolerance and Iteration Limits ⬜
**File**: `BrachistochroneProblemSolver.cs:157-166`

**Issue**: Current settings (tolerance=1e-2, maxIter=200) may be insufficient.

**Actions**:
- [ ] Test with relaxed tolerance: 1e-1, 5e-2
- [ ] Test with increased iterations: 500, 1000
- [ ] Monitor convergence rate (gradient norm vs iteration)
- [ ] Check if optimizer is making progress but slowly

### 5.2 L-BFGS Configuration ⬜
**File**: `LBFGSOptimizer.cs`

**Issue**: L-BFGS may struggle with the problem structure.

**Actions**:
- [ ] Test different L-BFGS memory sizes: 5, 10, 20
- [ ] Try alternative optimizers:
  - ConjugateGradientOptimizer
  - GradientDescentOptimizer (for sanity check)
- [ ] Enable more verbose logging from L-BFGS
- [ ] Monitor line search failures

### 5.3 Augmented Lagrangian Settings ⬜
**File**: `HermiteSimpsonSolver.cs:605-614`

**Issue**: The outer augmented Lagrangian optimizer may have penalty parameter issues.

**Actions**:
- [ ] Add logging to AugmentedLagrangianOptimizer
- [ ] Monitor penalty parameters and Lagrange multipliers
- [ ] Check constraint violation trends
- [ ] Test with different initial penalty parameters

**Expected Output**: Identify if constraint handling is causing issues

---

## Phase 6: Numerical Conditioning ⬜

### 6.1 Variable Scaling Analysis ⬜

**Issue**: T_f (order 1-5) mixed with position (order 0-10), velocity (order 0-20) may cause poor conditioning.

**Actions**:
- [ ] Compute condition number of Jacobian at initial point
- [ ] Test problem with normalized states: x̃ = x/10, ỹ = y/10, ṽ = v/20, T̃_f = T_f/2
- [ ] Adjust bounds accordingly for normalized problem
- [ ] Compare convergence with normalized vs unnormalized

### 6.2 Time-Scaling Formulation Alternatives ⬜

**Issue**: The current time-scaling may have inherent numerical issues.

**Actions**:
- [ ] Test alternative: Use T_f as a separate optimization parameter (not a state)
- [ ] Test alternative: Use terminal cost Φ = T_f instead of running cost
- [ ] Compare with direct formulation (no time-scaling, fixed final time)
- [ ] Review literature for best practices in free final time problems

**Expected Output**: Identify if time-scaling formulation itself is problematic

---

## Phase 7: Defect Constraint Analysis ⬜

### 7.1 Defect Monitoring ⬜
**File**: `HermiteSimpsonSolver.cs:549-602`

**Issue**: Defects may not be reducing, especially for T_f component.

**Actions**:
- [ ] Log maximum defect per state component: [x, y, v, T_f]
- [ ] Check if T_f defects are particularly large
- [ ] Monitor defect evolution across iterations
- [ ] Verify defect gradient computation for T_f

### 7.2 Collocation Scheme Investigation ⬜

**Issue**: Hermite-Simpson may not be ideal for this problem structure.

**Actions**:
- [ ] Test with different number of segments: 10, 20, 30, 50
- [ ] Try Legendre-Gauss-Lobatto solver instead
- [ ] Compare defect residuals between solvers
- [ ] Check if mesh refinement helps (currently disabled in headless mode)

**Expected Output**: Determine if transcription method affects convergence

---

## Phase 8: Physics Validation ⬜

### 8.1 Fixed Final Time Comparison ⬜

**Actions**:
- [ ] Run fixed final time version with T_f = T_optimal from successful solve
- [ ] Compare state trajectories
- [ ] Verify free final time formulation produces same trajectory
- [ ] Check if cost values match

### 8.2 Simplified Test Case ⬜

**Actions**:
- [ ] Create simpler geometry: shorter distance, smaller height difference
- [ ] Test with v0 = 0 exactly (not 1e-6)
- [ ] Try with initial velocity guess v0 = √(2gh) for free fall estimate
- [ ] Verify convergence on simplified case before full problem

**Expected Output**: Identify if geometry or initial conditions cause issues

---

## Phase 9: Diagnostic Instrumentation ⬜

### 9.1 Add Comprehensive Logging ⬜

**Actions**:
- [ ] Create diagnostics branch with extensive logging
- [ ] Log at each major stage:
  - Initial guess creation
  - Initial objective and constraint evaluation
  - Each optimizer iteration (state norms, cost, gradient norm, max defect)
  - Line search details
  - Final convergence status
- [ ] Save full state trajectory at each iteration to file
- [ ] Create visualization of convergence history

### 9.2 Create Minimal Reproducible Example ⬜

**Actions**:
- [ ] Extract problem into standalone test case
- [ ] Create unit test that demonstrates non-convergence
- [ ] Compare with known-good formulation (e.g., published Brachistochrone implementations)
- [ ] Document exact failure mode (oscillation, divergence, stall)

**Expected Output**: Clear characterization of failure mode

---

## Phase 10: Literature Review and Reference Implementation ⬜

### 10.1 Compare with Reference Implementations ⬜

**Actions**:
- [ ] Review GPOPS-II, CasADi, or other optimal control libraries
- [ ] Find published Brachistochrone free final time examples
- [ ] Compare formulation choices:
  - Time-scaling approach
  - Cost function structure
  - Constraint formulation
- [ ] Identify differences from working implementations

### 10.2 Theoretical Analysis ⬜

**Actions**:
- [ ] Review Pontryagin necessary conditions for this problem
- [ ] Verify transversality condition for free final time: H(t_f) = 0
- [ ] Check if time-scaling satisfies optimality conditions
- [ ] Consult optimal control textbooks (Betts, Ross, etc.)

**Expected Output**: Theoretical validation or identification of formulation error

---

## Priority Execution Order

Based on likely root causes, execute in this order:

1. **Phase 1** - Verify gradients are correct (most common source of issues)
2. **Phase 3.1** - Test different T_f initial guesses (quick diagnostic)
3. **Phase 5.1** - Relax tolerances and increase iterations (rule out premature termination)
4. **Phase 9.1** - Add logging to understand failure mode
5. **Phase 6.2** - Try alternative formulation (terminal cost instead of running cost)
6. **Phase 7.1** - Monitor defects to see which constraints fail
7. **Phase 8.2** - Test simplified geometry
8. **Remaining phases** - Based on findings from above

---

## Success Criteria

- [ ] Optimizer converges to local minimum (gradient norm < tolerance)
- [ ] Maximum defect < tolerance (dynamics satisfied)
- [ ] Boundary conditions satisfied within tolerance
- [ ] Final T_f matches integrated cost
- [ ] Solution is physically plausible (cycloid-like trajectory)
- [ ] Reproducible across multiple runs with different initial guesses

---

## Notes and Observations

*Use this section to record findings as you work through the plan*

### Findings:

**Phase 1 Results** (✅ COMPLETED):
- All time-scaled gradient computations verified numerically
- Chain rule ∂(T_f·f)/∂x = T_f·(∂f/∂x) and ∂(T_f·f)/∂T_f = f is correctly implemented
- Running cost L = T_f has correct gradient [0, 0, 0, 1, 0, 0]
- Integrated cost matches T_f within numerical precision
- **Gradient computations are NOT the issue**

---

## References

- Betts, J. T. (2010). Practical Methods for Optimal Control and Estimation Using Nonlinear Programming (2nd ed.)
- Ross, I. M. (2015). A Primer on Pontryagin's Principle in Optimal Control
- Kelly, M. (2017). An Introduction to Trajectory Optimization: How to Do Your Own Direct Collocation
