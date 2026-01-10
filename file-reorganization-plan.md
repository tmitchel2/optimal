# File Reorganization Plan

This document tracks the planned reorganization of C# files in the Optimal project.

## Status Legend

- `pending` - Not yet analyzed
- `analyzed` - Analyzed by name-categorize agent
- `approved` - Approved for move/rename
- `completed` - Move/rename completed
- `no-change` - No changes needed

## Files

| # | Status | Current Path | Target Path | Notes |
|---|--------|--------------|-------------|-------|
| 1 | no-change | NonLinear/StoppingReason.cs | NonLinear/StoppingReason.cs | Enum defines termination conditions; appropriately placed alongside OptimizerResult, ConvergenceMonitor, IOptimizer |
| 2 | completed | NonLinear/IOptimizer.cs | NonLinear.Unconstrained/IOptimizer.cs | Interface for unconstrained nonlinear optimization; uses dot-notation per project conventions |
| 3 | no-change | NonLinear/ConjugateGradientFormula.cs | NonLinear/ConjugateGradientFormula.cs | Enum for CG beta formulas; correctly placed alongside ConjugateGradientOptimizer |
| 4 | no-change | NonLinear/LBFGSMemory.cs | NonLinear/LBFGSMemory.cs | Core data structure for L-BFGS; belongs alongside LBFGSOptimizer |
| 5 | no-change | NonLinear/TwoLoopRecursion.cs | NonLinear/TwoLoopRecursion.cs | L-BFGS internal implementation; stays with LBFGSMemory per flat folder rule |
| 6 | no-change | NonLinear/OptimizerResult.cs | NonLinear/OptimizerResult.cs | Core type for optimization results; correctly placed with IOptimizer and related types |
| 7 | no-change | NonLinear/ConvergenceMonitor.cs | NonLinear/ConvergenceMonitor.cs | Core monitoring component; belongs alongside IOptimizer, OptimizerResult, StoppingReason |
| 8 | no-change | NonLinear/GradientDescentOptimizer.cs | NonLinear/GradientDescentOptimizer.cs | First-order unconstrained optimizer; matches peer optimizers location |
| 9 | no-change | NonLinear/ConjugateGradientOptimizer.cs | NonLinear/ConjugateGradientOptimizer.cs | Unconstrained optimizer; correctly placed with peer implementations |
| 10 | completed | NonLinear/LBFGSOptimizer.cs | NonLinear.Unconstrained/LBFGSOptimizer.cs | Quasi-Newton unconstrained method; follows dot-separated folder pattern |
| 11 | completed | NonLinear/AugmentedLagrangianOptimizer.cs | NonLinear.Constrained/AugmentedLagrangianOptimizer.cs | Constrained optimization technique; distinct from unconstrained optimizers |
| 12 | no-change | Control.Collocation/CollocationGrid.cs | Control.Collocation/CollocationGrid.cs | Foundational component for collocation methods; appropriately placed |
| 13 | no-change | Control.Collocation/HermiteSimpsonTranscription.cs | Control.Collocation/HermiteSimpsonTranscription.cs | Collocation transcription method; correctly grouped with other transcriptions |
| 14 | no-change | Control.Collocation/ILGLTranscription.cs | Control.Collocation/ILGLTranscription.cs | Interface for LGL transcription; appropriately in collocation folder |
| 15 | no-change | Control.Collocation/LegendreGaussLobatto.cs | Control.Collocation/LegendreGaussLobatto.cs | Quadrature infrastructure for collocation; maintains cohesion with consumers |
| 16 | no-change | Control.Collocation/LegendreGaussLobattoTranscription.cs | Control.Collocation/LegendreGaussLobattoTranscription.cs | LGL transcription method; correctly placed with related LGL components |
| 17 | no-change | Control.Collocation/MeshRefinement.cs | Control.Collocation/MeshRefinement.cs | Adaptive mesh refinement for collocation; appropriately placed |
| 18 | no-change | Control.Collocation/ParallelLGLTranscription.cs | Control.Collocation/ParallelLGLTranscription.cs | Parallel LGL variant; correctly placed with other transcriptions |
| 19 | completed | Control.Collocation/ParallelTranscription.cs | Control.Collocation/ParallelHermiteSimpsonTranscription.cs | RENAME: Generic name should match pattern of ParallelLGLTranscription |
| 20 | completed | Control.Core/CollocationResult.cs | Control.Collocation/CollocationResult.cs | MOVE: Collocation-specific result type belongs with collocation methods |
| 21 | no-change | Control.Core/ControlProblem.cs | Control.Core/ControlProblem.cs | Core problem definition; foundational type used by all solvers |
| 22 | no-change | Control.Core/ISolver.cs | Control.Core/ISolver.cs | Core interface for control solvers; correctly in Core with related types |
| 23 | no-change | Control.Core/MultiPhaseControlProblem.cs | Control.Core/MultiPhaseControlProblem.cs | Core multi-phase definitions; appropriately placed in Core |
| 24 | no-change | Control.Core/ProgressCallback.cs | Control.Core/ProgressCallback.cs | Progress reporting for control iterations; stays in Control.Core with related types |
| 25 | completed | Control.Indirect/AdvancedPontryaginSolver.cs | Control.Indirect/MultipleShootingPontryaginSolver.cs | RENAME: Better describes algorithm (multiple shooting + Newton + continuation) |
| 26 | completed | Control.Indirect/MultipleShootingSolver.cs | Control.Solvers/MultipleShootingSolver.cs | MOVE: Direct transcription method; belongs with other solvers not indirect methods |
| 27 | no-change | Control.Indirect/PontryaginSolver.cs | Control.Indirect/PontryaginSolver.cs | True indirect method based on Pontryagin's Minimum Principle |
| 28 | no-change | Control.Optimization/AutoDiffGradientHelper.cs | Control.Optimization/AutoDiffGradientHelper.cs | Optimization gradient helper; appropriately placed |
| 29 | no-change | Control.Optimization/AutoDiffLGLGradientHelper.cs | Control.Optimization/AutoDiffLGLGradientHelper.cs | LGL-specific gradient helper; correctly placed |
| 30 | no-change | Control.Optimization/ConstraintConfigurator.cs | Control.Optimization/ConstraintConfigurator.cs | Constraint configuration helper; appropriately placed |
| 31 | completed | Control.Optimization/LQRInitializer.cs | Control.Initialization/LQRInitializer.cs | MOVE: Initialization strategy; clearer grouping for init methods |
| 32 | no-change | Control.Optimization/NumericalGradients.cs | Control.Optimization/NumericalGradients.cs | Numerical gradients utility; stays to avoid single-file folder |
| 33 | no-change | Control.Optimization/ObjectiveFunctionFactory.cs | Control.Optimization/ObjectiveFunctionFactory.cs | Objective function factory; appropriately placed |
| 34 | completed | Control.Optimization/WarmStart.cs | Control.Collocation/WarmStart.cs | MOVE: Collocation-specific warm start; belongs with collocation types |
| 35 | no-change | Control.Solvers/ContinuationSolver.cs | Control.Solvers/ContinuationSolver.cs | Continuation/homotopy solver; correctly placed |
| 36 | no-change | Control.Solvers/LegendreGaussLobattoSolver.cs | Control.Solvers/LegendreGaussLobattoSolver.cs | Direct transcription solver; correctly placed with other solvers |
| 37 | no-change | Control.Solvers/MultiPhaseSolver.cs | Control.Solvers/MultiPhaseSolver.cs | Multi-phase solver; correctly placed with other solvers |
| 38 | no-change | Control.Solvers/HermiteSimpsonSolver.cs | Control.Solvers/HermiteSimpsonSolver.cs | Hermite-Simpson solver; correctly placed |
| 39 | no-change | NonLinear.Constraints/BoxConstraints.cs | NonLinear.Constraints/BoxConstraints.cs | Dot-notation is correct per project conventions; no change needed |
| 40 | no-change | NonLinear.Constraints/EqualityConstraint.cs | NonLinear.Constraints/EqualityConstraint.cs | Equality constraint; appropriately placed with constraints |
| 41 | no-change | NonLinear.Constraints/IConstraint.cs | NonLinear.Constraints/IConstraint.cs | Dot-notation is correct per project conventions; no change needed |
| 42 | no-change | NonLinear.Constraints/InequalityConstraint.cs | NonLinear.Constraints/InequalityConstraint.cs | Inequality constraint; appropriately placed |
| 43 | no-change | NonLinear.LineSearch/BacktrackingLineSearch.cs | NonLinear.LineSearch/BacktrackingLineSearch.cs | Backtracking line search; correctly placed |
| 44 | no-change | NonLinear.LineSearch/ILineSearch.cs | NonLinear.LineSearch/ILineSearch.cs | Line search interface; appropriately placed |
| 45 | no-change | NonLinear.LineSearch/ParallelBacktrackingLineSearch.cs | NonLinear.LineSearch/ParallelBacktrackingLineSearch.cs | Parallel line search; correctly placed |

## Progress

- Total files: 45
- Completed: 9 (7 moves + 2 renames)
- No change: 36
- Pending: 0

**Status: COMPLETED** - All planned moves and renames have been executed successfully. Build verified.

## Summary of Completed Changes

**Validated against `developing-csharp-code` skill guidance:**
- Folders should be flat and only one level deep
- Use dots (`.`) to separate folder names rather than nested directories

### Files to Move (7 files)

| Current Path | Target Path |
|--------------|-------------|
| NonLinear/IOptimizer.cs | NonLinear.Unconstrained/IOptimizer.cs |
| NonLinear/LBFGSOptimizer.cs | NonLinear.Unconstrained/LBFGSOptimizer.cs |
| NonLinear/AugmentedLagrangianOptimizer.cs | NonLinear.Constrained/AugmentedLagrangianOptimizer.cs |
| Control.Core/CollocationResult.cs | Control.Collocation/CollocationResult.cs |
| Control.Indirect/MultipleShootingSolver.cs | Control.Solvers/MultipleShootingSolver.cs |
| Control.Optimization/LQRInitializer.cs | Control.Initialization/LQRInitializer.cs |
| Control.Optimization/WarmStart.cs | Control.Collocation/WarmStart.cs |

### Files to Rename (2 files)

| Current Name | New Name |
|--------------|----------|
| Control.Collocation/ParallelTranscription.cs | Control.Collocation/ParallelHermiteSimpsonTranscription.cs |
| Control.Indirect/AdvancedPontryaginSolver.cs | Control.Indirect/MultipleShootingPontryaginSolver.cs |

### Corrections Made During Validation

| Original Recommendation | Correction | Reason |
|------------------------|------------|--------|
| NonLinear/IOptimizer.cs → NonLinear/Unconstrained/ | → NonLinear.Unconstrained/ | Use dot-notation, not nested folders |
| NonLinear/TwoLoopRecursion.cs → deeply nested path | No change | 4-level nesting violates flat folder rule; stays with LBFGSMemory |
| Control.Core/ProgressCallback.cs → Iteration/ | No change | Creating standalone folder is inconsistent; stays in Control.Core |
| NonLinear.Constraints/BoxConstraints.cs → nested path | No change | Current dot-notation is correct per project conventions |
| NonLinear.Constraints/IConstraint.cs → nested path | No change | Current dot-notation is correct per project conventions |
| Control.Optimization/NumericalGradients.cs → NonLinear.Differentiation/ | No change | Keep in Control.Optimization; avoid creating single-file folder |

### Files Unchanged (36 files)

The remaining 36 files are appropriately placed and require no changes.

## Batch History

### Batch 1 (Files 1-5) - Completed

| File | Recommendation |
|------|----------------|
| StoppingReason.cs | No change - appropriately placed |
| IOptimizer.cs | Move to NonLinear/Unconstrained/ |
| ConjugateGradientFormula.cs | No change - appropriately placed |
| LBFGSMemory.cs | No change - appropriately placed |
| TwoLoopRecursion.cs | Move to NonLinear/Unconstrained/QuasiNewton/LBFGS/ |

### Batch 2 (Files 6-10) - Completed

| File | Recommendation |
|------|----------------|
| OptimizerResult.cs | No change - core optimization result type |
| ConvergenceMonitor.cs | No change - core monitoring component |
| GradientDescentOptimizer.cs | No change - matches peer optimizers |
| ConjugateGradientOptimizer.cs | No change - correctly placed |
| LBFGSOptimizer.cs | Move to NonLinear.Unconstrained/ |

### Batch 3 (Files 11-15) - Completed

| File | Recommendation |
|------|----------------|
| AugmentedLagrangianOptimizer.cs | Move to NonLinear.Constrained/ |
| CollocationGrid.cs | No change - foundational collocation component |
| HermiteSimpsonTranscription.cs | No change - correctly grouped |
| ILGLTranscription.cs | No change - appropriately placed |
| LegendreGaussLobatto.cs | No change - maintains cohesion |

### Batch 4 (Files 16-20) - Completed

| File | Recommendation |
|------|----------------|
| LegendreGaussLobattoTranscription.cs | No change - correctly placed |
| MeshRefinement.cs | No change - appropriately placed |
| ParallelLGLTranscription.cs | No change - correctly placed |
| ParallelTranscription.cs | Rename to ParallelHermiteSimpsonTranscription.cs |
| CollocationResult.cs | Move to Control.Collocation/ |

### Batch 5 (Files 21-25) - Completed

| File | Recommendation |
|------|----------------|
| ControlProblem.cs | No change - foundational type |
| ISolver.cs | No change - core interface |
| MultiPhaseControlProblem.cs | No change - appropriately placed |
| ProgressCallback.cs | Move to Iteration/ |
| AdvancedPontryaginSolver.cs | Rename to MultipleShootingPontryaginSolver.cs |

### Batch 6 (Files 26-30) - Completed

| File | Recommendation |
|------|----------------|
| MultipleShootingSolver.cs | Move to Control.Solvers/ |
| PontryaginSolver.cs | No change - true indirect method |
| AutoDiffGradientHelper.cs | No change - appropriately placed |
| AutoDiffLGLGradientHelper.cs | No change - correctly placed |
| ConstraintConfigurator.cs | No change - appropriately placed |

### Batch 7 (Files 31-35) - Completed

| File | Recommendation |
|------|----------------|
| LQRInitializer.cs | Move to Control.Initialization/ |
| NumericalGradients.cs | Move to NonLinear.Differentiation/ |
| ObjectiveFunctionFactory.cs | No change - appropriately placed |
| WarmStart.cs | Move to Control.Collocation/ |
| ContinuationSolver.cs | No change - correctly placed |

### Batch 8 (Files 36-40) - Completed

| File | Recommendation |
|------|----------------|
| LegendreGaussLobattoSolver.cs | No change - correctly placed |
| MultiPhaseSolver.cs | No change - correctly placed |
| HermiteSimpsonSolver.cs | No change - correctly placed |
| BoxConstraints.cs | Consider nested folders (NonLinear/Constraints/) |
| EqualityConstraint.cs | No change - appropriately placed |

### Batch 9 (Files 41-45) - Completed

| File | Recommendation |
|------|----------------|
| IConstraint.cs | Consider nested folders (NonLinear/Constraints/) |
| InequalityConstraint.cs | No change - appropriately placed |
| BacktrackingLineSearch.cs | No change - correctly placed |
| ILineSearch.cs | No change - appropriately placed |
| ParallelBacktrackingLineSearch.cs | No change - correctly placed |
