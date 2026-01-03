# Brachistochrone Solver Investigation Plan

## Executive Summary

The BrachistochroneProblemSolver has issues with multiple solver/variant combinations. This document provides a detailed step-by-step plan to systematically diagnose and fix each issue, with unit tests for each component of the solving process.

### Current Status Matrix

| Variant | Hermite-Simpson | LGL |
|---------|-----------------|-----|
| Fixed Final Time | ✅ Works (2 iters) | ❌ Fails (doesn't converge) |
| Free Final Time (Terminal Cost) | ⚠️ Oscillates (14 iters), correct result | ❌ Converges but wrong solution (33% energy error) |
| Free Final Time (Running Cost) | ⚠️ Oscillates (15 iters), correct result | ❌ Converges but wrong solution (33% energy error) |

### Key Observations

**LGL Failure Pattern**: In all LGL cases, the control θ stays fixed at π/2 (initial guess) and T_f stays at 1.8 (initial guess). The trajectory shows almost no movement until τ=0.8-0.9, then jumps to the final position. This indicates the optimizer is not receiving meaningful gradients for control or T_f variables.

**Hermite-Simpson Oscillation Pattern**: Violation oscillates after reaching ~0.02-0.05, bouncing up and down before finally converging. The final result is physically correct (energy conservation within 0.1%).

## Observed Issues

### Issue 1: LGL Fixed Time - Complete Failure
- **Symptom**: Constraint violation never decreases below ~2.0-4.0
- **Outer iterations**: 100 (max) with no convergence
- **Max defect**: Never drops below several units
- **Cost**: Stays at 1.8 (the fixed time)
- **Observation**: Initial defect is 9.8 (gravity value), suggesting dynamics-related issue

### Issue 2: LGL Free Time - Wrong Solution
- **Symptom**: "Converges" but with physically impossible trajectory
- **Energy conservation error**: 33.1% (final velocity 6.62 m/s vs expected 9.90 m/s)
- **Control stuck**: θ = π/2 throughout (straight down), never optimizes
- **Trajectory**: Bead barely moves until τ=0.8, then jumps to final position
- **Root cause hypothesis**: LGL gradient computation for time-scaled dynamics is incorrect

### Issue 3: Hermite-Simpson Free Time - Oscillation
- **Symptom**: Converges but takes 14 iterations with oscillating violation
- **Violation pattern**: 3.83 → 1.01 → 0.095 → 0.019 → **0.035** → 0.029 → 0.025 → **0.046** → ...
- **Root cause hypothesis**: Augmented Lagrangian penalty scaling or gradient inconsistency
- **Final result**: Correct (energy error 0.0%), but slow convergence

---

## Phase 1: Foundation Tests - Dynamics & Gradients Verification

### Task 1.1: Verify Brachistochrone Dynamics Correctness
- [ ] **Test**: Physical dynamics values (XRate, YRate, VRate) for known inputs
- [ ] **Test**: Verify AutoDiff gradients match numerical finite differences
- [ ] **Test**: Time-scaled dynamics (4-state version) values and gradients
- [ ] **File**: `src/Optimal.Tests/Problems.Brachistochrone.Tests/BrachistochroneDynamicsVerificationTests.cs`

### Task 1.2: Verify Running Cost Gradients
- [ ] **Test**: L = 1 for fixed time (trivial gradient = 0)
- [ ] **Test**: L = T_f for free time running cost variant (∂L/∂T_f = 1)
- [ ] **Test**: Numerical vs analytical gradient comparison
- [ ] **File**: Same as 1.1

### Task 1.3: Verify Terminal Cost Gradients  
- [ ] **Test**: Φ = T_f for free time terminal cost variant (∂Φ/∂T_f = 1)
- [ ] **Test**: Numerical vs analytical gradient comparison
- [ ] **File**: Same as 1.1

---

## Phase 2: Transcription Layer Tests

### Task 2.1: Hermite-Simpson Transcription Tests
- [ ] **Test**: Decision vector layout correctness (state/control extraction)
- [ ] **Test**: Defect computation for known trajectory (sine wave)
- [ ] **Test**: Defect should be zero for dynamically feasible trajectory
- [ ] **Test**: Running cost integration accuracy (Simpson's rule)
- [ ] **Test**: Terminal cost evaluation at correct final state
- [ ] **File**: Extend `src/Optimal.Tests/Control.Tests/HermiteSimpsonTranscriptionTests.cs`

### Task 2.2: LGL Transcription Tests
- [ ] **Test**: Decision vector layout correctness (global point indexing)
- [ ] **Test**: LGL points and weights correctness
- [ ] **Test**: Differentiation matrix accuracy (D * polynomial = derivative)
- [ ] **Test**: Defect computation for known polynomial trajectory
- [ ] **Test**: Running cost integration accuracy (LGL quadrature)
- [ ] **Test**: Defect should be zero for dynamically feasible polynomial
- [ ] **File**: Extend `src/Optimal.Tests/Control.Tests/LegendreGaussLobattoTranscriptionTests.cs`

### Task 2.3: Time-Scaling Transcription Tests
- [ ] **Test**: Verify τ ∈ [0,1] → t ∈ [0, T_f] mapping
- [ ] **Test**: Verify defect computation with T_f as state variable
- [ ] **Test**: Verify T_f state dynamics (dT_f/dτ = 0)
- [ ] **File**: New file `src/Optimal.Tests/Control.Tests/TimeScalingTranscriptionTests.cs`

---

## Phase 3: Gradient Helper Tests

### Task 3.1: AutoDiffGradientHelper Tests (Hermite-Simpson)
- [ ] **Test**: Running cost gradient computation
- [ ] **Test**: Terminal cost gradient computation
- [ ] **Test**: Defect gradient computation
- [ ] **Test**: Compare all analytical gradients to numerical finite differences
- [ ] **File**: New file `src/Optimal.Tests/Control.Tests/AutoDiffGradientHelperTests.cs`

### Task 3.2: AutoDiffLGLGradientHelper Tests
- [ ] **Test**: Running cost gradient for LGL quadrature
- [ ] **Test**: Terminal cost gradient
- [ ] **Test**: Defect gradient at interior points
- [ ] **Test**: Compare all analytical gradients to numerical finite differences
- [ ] **Critical**: Verify gradient indexing matches decision vector layout
- [ ] **File**: New file `src/Optimal.Tests/Control.Tests/AutoDiffLGLGradientHelperTests.cs`

### Task 3.3: Gradient Consistency for Time-Scaled Problems
- [ ] **Test**: Gradient w.r.t. T_f in dynamics (∂f̃/∂T_f = f)
- [ ] **Test**: Gradient w.r.t. T_f in running cost
- [ ] **Test**: Gradient w.r.t. T_f in terminal cost
- [ ] **Test**: Full numerical vs analytical comparison for Brachistochrone
- [ ] **File**: New file `src/Optimal.Tests/Control.Tests/TimeScaledGradientTests.cs`

---

## Phase 4: Solver Unit Tests

### Task 4.1: Simple Integrator Problem Tests
- [ ] **Test**: ẋ = u, min ∫u²dt, fixed time (both solvers)
- [ ] **Test**: ẋ = u, min ∫u²dt, free time variant
- [ ] **Test**: Known analytical solution comparison
- [ ] **File**: Extend existing solver test files

### Task 4.2: Brachistochrone-Specific Solver Tests

#### 4.2.1: Hermite-Simpson + Fixed Time
- [ ] **Test**: Convergence in ≤5 outer iterations
- [ ] **Test**: Max defect < 1e-2
- [ ] **Test**: Final position accuracy (x_f, y_f)
- [ ] **Test**: Energy conservation (v_f matches √(2g·Δh))
- [ ] **File**: New file `src/Optimal.Tests/Problems.Brachistochrone.Tests/BrachistochroneSolverTests.cs`

#### 4.2.2: Hermite-Simpson + Free Time
- [ ] **Test**: Convergence (should not oscillate excessively)
- [ ] **Test**: Track violation history to detect oscillation
- [ ] **Test**: Final T_f is near theoretical optimal (~1.8s)
- [ ] **Test**: Energy conservation
- [ ] **Investigation**: If oscillation detected, analyze penalty update strategy

#### 4.2.3: LGL + Fixed Time
- [ ] **Test**: Convergence with Order=3 (simpler case)
- [ ] **Test**: Convergence with Order=5 (current failing case)
- [ ] **Test**: Debug: Log defect per state component
- [ ] **Investigation**: Identify which state component has largest defect

#### 4.2.4: LGL + Free Time
- [ ] **Test**: Control should NOT be constant π/2
- [ ] **Test**: Energy conservation must be < 5%
- [ ] **Test**: T_f should be optimized (not stuck at initial guess)
- [ ] **Investigation**: Check if T_f state is being updated by optimizer

---

## Phase 5: Root Cause Analysis

### Task 5.1: Investigate LGL Fixed Time Failure

**Hypothesis 1: Defect gradient indexing error**
- LGL uses interior points only for defects
- Global point index calculation may be off
- Decision vector offset may not match gradient helper

**Investigation Steps:**
1. [ ] Create minimal 2-segment, order-3 LGL problem
2. [ ] Print decision vector layout explicitly
3. [ ] Compute defect numerically and analytically
4. [ ] Compare defect gradient element-by-element
5. [ ] Identify index mismatch

**Hypothesis 2: Time scaling in LGL transcription**
- LGL uses τ ∈ [-1, 1] per segment, not physical time
- Time conversion may be incorrect for dynamics evaluation

### Task 5.2: Investigate LGL Free Time Wrong Solution

**Hypothesis 1: T_f not being updated**
- Control θ stuck at π/2 suggests optimizer not exploring
- T_f staying at 1.8 suggests gradient w.r.t. T_f may be zero or wrong sign

**Investigation Steps:**
1. [ ] Log gradient w.r.t. T_f elements during optimization
2. [ ] Verify ∂(cost)/∂T_f > 0 (should push T_f up if velocity too low)
3. [ ] Check defect constraint ∂(defect)/∂T_f
4. [ ] Verify initial guess for control is reasonable

**Hypothesis 2: Constraint Jacobian sparsity**
- LGL has different sparsity pattern than HS
- Defect at point i depends on all states in segment

### Task 5.3: Investigate Hermite-Simpson Oscillation

**Hypothesis 1: Augmented Lagrangian penalty scaling**
- Penalty μ may be increasing too aggressively
- Causes optimizer to over-correct constraints

**Investigation Steps:**
1. [ ] Log μ value and violation at each outer iteration
2. [ ] Check if violation oscillates when μ crosses threshold
3. [ ] Test with slower μ scaling (e.g., ×2 instead of ×10)

**Hypothesis 2: Gradient inconsistency**
- Mixing analytical and numerical gradients
- Small errors accumulate

---

## Phase 6: Fixes and Validation

### Task 6.1: LGL Defect Gradient Fix
- [ ] Fix identified indexing errors
- [ ] Add explicit tests for corrected code
- [ ] Run full brachistochrone solver test

### Task 6.2: LGL Time-Scaled Dynamics Fix
- [ ] Fix T_f gradient propagation
- [ ] Verify dynamics gradient includes ∂f̃/∂T_f = f_physical
- [ ] Run free time solver test

### Task 6.3: Hermite-Simpson Oscillation Mitigation
- [ ] Tune augmented Lagrangian parameters
- [ ] Consider trust-region inner optimizer
- [ ] Add convergence monitoring to detect oscillation

### Task 6.4: Final Integration Tests
- [ ] All 4 combinations pass: HS×{Fixed,Free} + LGL×{Fixed,Free}
- [ ] Running cost variant also passes
- [ ] Performance: HS should converge in ≤10 iterations
- [ ] Performance: LGL should converge in ≤15 iterations

---

## Test File Organization

```
src/Optimal.Tests/
├── Control.Tests/
│   ├── AutoDiffGradientHelperTests.cs          (NEW - Task 3.1)
│   ├── AutoDiffLGLGradientHelperTests.cs       (NEW - Task 3.2)
│   ├── TimeScalingTranscriptionTests.cs        (NEW - Task 2.3)
│   ├── TimeScaledGradientTests.cs              (NEW - Task 3.3)
│   ├── HermiteSimpsonTranscriptionTests.cs     (EXTEND - Task 2.1)
│   ├── LegendreGaussLobattoTranscriptionTests.cs (EXTEND - Task 2.2)
│   ├── HermiteSimpsonSolverTests.cs            (EXTEND - Task 4.1)
│   └── LegendreGaussLobattoSolverTests.cs      (EXTEND - Task 4.1)
└── Problems.Brachistochrone.Tests/
    ├── BrachistochroneDynamicsTests.cs         (EXISTS)
    ├── BrachistochroneDynamicsVerificationTests.cs (NEW - Task 1.1-1.3)
    └── BrachistochroneSolverTests.cs           (NEW - Task 4.2)
```

---

## Execution Order

1. **Phase 1** - Verify foundation is correct (dynamics, gradients)
2. **Phase 2** - Verify transcription layer
3. **Phase 3** - Verify gradient helpers
4. **Phase 4** - Create solver-level tests that currently fail
5. **Phase 5** - Use failing tests to diagnose root causes
6. **Phase 6** - Fix issues and verify all tests pass

---

## Success Criteria

1. All unit tests pass
2. HS Fixed Time: Converges in ≤5 iterations, defect < 1e-3
3. HS Free Time: Converges in ≤10 iterations, no oscillation (monotonic decrease after iter 3)
4. LGL Fixed Time: Converges in ≤15 iterations, defect < 1e-3
5. LGL Free Time: Converges in ≤15 iterations, energy error < 1%
6. All variants achieve final position within 0.1m of target
7. Energy conservation error < 1% for all variants

---

## Notes

### Key Insight from Analysis

The LGL Free Time "convergence" shows control stuck at π/2 (initial guess) and trajectory only moving at the end. This strongly suggests:

1. The optimizer is not receiving correct gradients for the control variables
2. The T_f state variable is not being properly optimized
3. The defect constraints may be satisfied at a local minimum that doesn't represent physical trajectory

### Why Hermite-Simpson Works Better

- Simpler defect structure (1 defect per segment vs Order-2 per segment)
- Defect depends on endpoints + midpoint only
- More local gradient structure
- Simpson's rule is well-conditioned

### LGL Challenges

- Each interior point defect depends on ALL states in segment via differentiation matrix
- Higher-order methods are more sensitive to gradient accuracy
- Spectral collocation can exhibit Runge phenomenon for poorly conditioned problems
