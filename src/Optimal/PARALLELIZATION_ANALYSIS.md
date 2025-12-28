# Parallelization Opportunities in Optimal

This document analyzes opportunities to parallelize computation in the Optimal library to improve solve performance for optimal control problems.

## Executive Summary

The Optimal library has **significant parallelization potential**, particularly in:
1. **Constraint/Gradient evaluation** (embarrassingly parallel)
2. **Multiple shooting** (independent interval solutions)
3. **Mesh refinement** (defect calculations)
4. **Continuation methods** (parameter sweep)
5. **Multi-phase problems** (independent phase initialization)

**Expected Impact**: 2-10x speedup on multi-core systems for medium to large problems.

---

## 1. Constraint and Gradient Evaluation

### Current Implementation
In `HermiteSimpsonSolver.cs` (lines 323-338), defect constraints are evaluated sequentially:

```csharp
// Build one constraint per defect (each segment × each state dimension)
for (var i = 0; i < totalDefects; i++)
{
    optimizer.WithEqualityConstraint(DefectConstraint(i));
}
```

Each defect constraint independently computes:
- Dynamics at interval endpoints
- Collocation point evaluation
- Hermite-Simpson interpolation error

### Parallelization Strategy
**Approach**: Parallel evaluation using `Parallel.For` or `PLINQ`

```csharp
// Compute ALL defects in parallel
var defects = new double[totalDefects];
var gradients = new double[totalDefects][];

Parallel.For(0, totalDefects, i =>
{
    var (value, gradient) = DefectConstraint(i)(z);
    defects[i] = value;
    gradients[i] = gradient;
});
```

**Benefits**:
- Defects are embarrassingly parallel (no shared state)
- Typical problems have 100-1000+ defects
- Linear speedup expected: ~4-8x on typical hardware

**Implementation Location**: 
- `HermiteSimpsonTranscription.cs`: `ComputeAllDefects()`
- Add parallel version: `ComputeAllDefectsParallel()`

---

## 2. Numerical Gradient Computation

### Current Implementation
`NumericalGradients.cs` computes gradients via finite differences sequentially:

```csharp
public static double[] ComputeGradient(Func<double[], double> f, double[] x)
{
    var gradient = new double[n];
    for (var i = 0; i < n; i++)
    {
        // Perturb dimension i
        x[i] += h;
        var fPlus = f(x);
        x[i] -= 2 * h;
        var fMinus = f(x);
        x[i] += h; // Restore
        gradient[i] = (fPlus - fMinus) / (2 * h);
    }
}
```

### Parallelization Strategy
**Approach**: Parallel dimension evaluation

```csharp
public static double[] ComputeGradientParallel(Func<double[], double> f, double[] x)
{
    var gradient = new double[n];
    
    Parallel.For(0, n, i =>
    {
        // Clone x for thread safety
        var xCopy = (double[])x.Clone();
        
        xCopy[i] += h;
        var fPlus = f(xCopy);
        xCopy[i] -= 2 * h;
        var fMinus = f(xCopy);
        
        gradient[i] = (fPlus - fMinus) / (2 * h);
    });
    
    return gradient;
}
```

**Benefits**:
- Perfect parallelization (2n independent function evaluations)
- Decision vectors typically 100-10,000 dimensions
- Expected speedup: ~4-8x on typical hardware

**Trade-offs**:
- Each thread needs memory copy (small overhead)
- Beneficial when n > ~50 dimensions
- Can add threshold: use parallel only if `n > parallelThreshold`

---

## 3. Multiple Shooting Solver

### Current Implementation
`MultipleShootingSolver.cs` solves shooting intervals sequentially:

```csharp
for (var i = 0; i < _shootingIntervals; i++)
{
    var intervalProblem = CreateIntervalProblem(i);
    var result = solver.Solve(intervalProblem);
    // Store result
}
```

### Parallelization Strategy
**Approach**: Parallel interval initialization, then iterative refinement

```csharp
// Phase 1: Initial solve (fully parallel)
var intervalResults = new CollocationResult[_shootingIntervals];

Parallel.For(0, _shootingIntervals, i =>
{
    var intervalProblem = CreateIntervalProblem(i);
    intervalResults[i] = solver.Solve(intervalProblem);
});

// Phase 2: Iteratively enforce continuity (parallel gradient evaluation)
while (!converged)
{
    // Compute continuity defects in parallel
    var defects = Parallel.For(0, _shootingIntervals - 1, i =>
    {
        return ComputeContinuityDefect(intervalResults[i], intervalResults[i + 1]);
    }).ToArray();
    
    // Update intervals to reduce defects
}
```

**Benefits**:
- Initial solve: N-way parallelism (N = number of intervals)
- Typical: 4-10 intervals → 4-10x speedup for initialization
- Continuity enforcement still benefits from parallel defect computation

**Challenges**:
- Intervals must be independent initially (good initial guess required)
- May need iterative outer loop (reduces parallel efficiency)

---

## 4. Mesh Refinement

### Current Implementation
`MeshRefinement.cs` evaluates defects sequentially to determine refinement:

```csharp
for (var i = 0; i < segments; i++)
{
    var defect = ComputeSegmentDefect(i);
    if (defect > threshold)
    {
        shouldRefine[i] = true;
    }
}
```

### Parallelization Strategy
**Approach**: Parallel defect evaluation

```csharp
var shouldRefine = new bool[segments];
var defects = new double[segments];

Parallel.For(0, segments, i =>
{
    defects[i] = ComputeSegmentDefect(i);
    shouldRefine[i] = defects[i] > threshold;
});
```

**Benefits**:
- Each segment defect is independent
- Typical problems: 20-200 segments
- Expected speedup: 4-8x

**Implementation**: 
- Modify `MeshRefinement.IdentifySegmentsToRefine()` method

---

## 5. Continuation Solver

### Current Implementation
`ContinuationSolver.cs` sequentially solves for each parameter value:

```csharp
for (var i = 0; i < parameterValues.Length; i++)
{
    problem.SetParameter(parameterValues[i]);
    results[i] = solver.Solve(problem);
    // Use previous solution as warm start for next
}
```

### Parallelization Strategy
**Approach**: Parallel parameter batches (when warm starting not critical)

```csharp
// Divide parameter range into batches
var batchSize = parameterValues.Length / numThreads;

Parallel.For(0, numThreads, threadId =>
{
    var startIdx = threadId * batchSize;
    var endIdx = Math.Min(startIdx + batchSize, parameterValues.Length);
    
    for (var i = startIdx; i < endIdx; i++)
    {
        problem.SetParameter(parameterValues[i]);
        results[i] = solver.Solve(problem);
        // Warm start within batch
    }
});
```

**Alternative**: Parallel prediction phase
- Compute all predictors in parallel
- Sequential correction with warm starting

**Benefits**:
- Good for parameter sweeps (sensitivity analysis)
- Typical: 10-100 parameter values
- Speedup: 2-4x (less than ideal due to warm start dependencies)

---

## 6. Multi-Phase Problems

### Current Implementation
`MultiPhaseSolver.cs` could parallelize phase-independent operations:

```csharp
// Initial solve of each phase (if linkages weak)
for (var i = 0; i < numPhases; i++)
{
    phaseResults[i] = SolvePhase(i, initialGuess);
}
```

### Parallelization Strategy
**Approach**: Parallel phase initialization

```csharp
// Solve all phases in parallel with initial guesses
var phaseResults = new CollocationResult[numPhases];

Parallel.For(0, numPhases, i =>
{
    phaseResults[i] = SolvePhase(i, initialGuess[i]);
});

// Then enforce linkage constraints iteratively
```

**Benefits**:
- Good for loosely coupled phases
- Typical: 2-5 phases
- Speedup: 2-5x for initialization

---

## 7. L-BFGS Optimizer

### Current Implementation
`LBFGSOptimizer.cs` has minimal parallelization (sequential two-loop recursion):

```csharp
var direction = TwoLoopRecursion.ComputeDirection(gradient, memory);
```

### Limited Parallelization Opportunities
The two-loop recursion is **inherently sequential** (data dependencies).

**Minor opportunities**:
1. Vector operations (dot products, scaling) - minimal benefit
2. Line search evaluations - could evaluate multiple trial points

**Recommendation**: Focus elsewhere (limited benefit, high complexity)

---

## Implementation Recommendations

### Priority 1: High Impact, Low Complexity
1. **Numerical gradients** - Add `ComputeGradientParallel()` with auto-threshold
2. **Defect constraints** - Parallelize `ComputeAllDefects()`
3. **Mesh refinement** - Parallel defect evaluation

### Priority 2: High Impact, Medium Complexity
4. **Multiple shooting** - Parallel interval initialization
5. **Continuation** - Parallel parameter batches

### Priority 3: Medium Impact, Medium Complexity
6. **Multi-phase** - Parallel phase initialization

---

## Configuration & API Design

### Proposed API

```csharp
// Global parallel configuration
public static class ParallelConfig
{
    public static bool EnableParallelization { get; set; } = true;
    public static int MaxDegreeOfParallelism { get; set; } = -1; // Use all cores
    public static int GradientParallelThreshold { get; set; } = 50; // Dimensions
    public static int DefectParallelThreshold { get; set; } = 20; // Segments
}

// Per-solver configuration
var solver = new HermiteSimpsonSolver()
    .WithSegments(100)
    .WithParallelization(true)  // Enable parallel constraint eval
    .WithMaxThreads(8);         // Limit thread count
```

### Environment Detection

```csharp
// Automatically disable parallelization if:
// 1. Single core system
// 2. Problem too small (overhead > benefit)
// 3. Debugging mode (for reproducibility)

if (Environment.ProcessorCount == 1 || 
    problem.DecisionVectorSize < ParallelConfig.GradientParallelThreshold)
{
    useParallel = false;
}
```

---

## Performance Measurement Plan

### Benchmarks to Create

1. **Gradient computation**:
   - Vary decision vector size: 10, 50, 100, 500, 1000, 5000
   - Compare sequential vs parallel
   - Measure overhead crossover point

2. **Defect evaluation**:
   - Vary segment count: 10, 20, 50, 100, 200
   - Compare sequential vs parallel
   - Measure on CartPole, Goddard Rocket

3. **Multiple shooting**:
   - Vary shooting intervals: 2, 4, 8, 16
   - Measure initialization time
   - Measure total solve time

4. **Full solve**:
   - Benchmark classic problems with/without parallelization
   - Report: wall time, CPU time, speedup factor

### Expected Results

| Problem | Sequential | Parallel (8 cores) | Speedup |
|---------|-----------|-------------------|---------|
| Small (n<100) | 1.0s | 0.9s | 1.1x (overhead) |
| Medium (n~500) | 10.0s | 2.5s | 4.0x |
| Large (n~2000) | 60.0s | 10.0s | 6.0x |
| CartPole | 5.0s | 1.5s | 3.3x |
| Goddard | 30.0s | 6.0s | 5.0x |

---

## Thread Safety Considerations

### Current State
- Most classes are **not thread-safe** (designed for single-threaded use)
- Stateful components: `LBFGSMemory`, `ConvergenceMonitor`, `CollocationGrid`

### Requirements for Parallelization

1. **Function evaluation**: Must be thread-safe
   - User-provided dynamics, costs, constraints
   - Document requirement clearly

2. **Memory isolation**: Each thread needs local state
   - Clone decision vectors for gradient evaluation
   - Use thread-local storage for intermediate results

3. **Result aggregation**: Thread-safe collection
   - Use `ConcurrentBag<T>` or pre-allocated arrays with indexed writes

### Documentation Updates

```csharp
/// <summary>
/// User-provided functions must be thread-safe when parallel evaluation is enabled.
/// Avoid shared mutable state in dynamics, cost, and constraint functions.
/// </summary>
```

---

## Implementation Phases

### Phase 1: Foundation (Week 1)
- [ ] Add `ParallelConfig` class
- [ ] Create `NumericalGradients.ComputeGradientParallel()`
- [ ] Add unit tests for parallel gradient computation
- [ ] Benchmark gradient computation

### Phase 2: Core Parallelization (Week 2)
- [ ] Parallelize `HermiteSimpsonTranscription.ComputeAllDefects()`
- [ ] Parallelize `MeshRefinement.IdentifySegmentsToRefine()`
- [ ] Update `HermiteSimpsonSolver` to use parallel versions
- [ ] Add configuration flags

### Phase 3: Advanced Methods (Week 3)
- [ ] Parallelize `MultipleShootingSolver` initialization
- [ ] Add parallel batch mode to `ContinuationSolver`
- [ ] Parallelize `MultiPhaseSolver` initialization

### Phase 4: Testing & Optimization (Week 4)
- [ ] Create comprehensive benchmarks
- [ ] Tune thresholds for parallel activation
- [ ] Add parallel solver tests
- [ ] Update documentation with performance guidance

---

## Risks & Mitigation

### Risk 1: Overhead > Benefit for Small Problems
**Mitigation**: Use thresholds; only parallelize when problem size exceeds crossover point

### Risk 2: User Functions Not Thread-Safe
**Mitigation**: Clear documentation; provide examples of thread-safe implementations

### Risk 3: Non-Deterministic Results (Debugging Difficulty)
**Mitigation**: Add deterministic mode flag; ensure parallel aggregation order doesn't affect results

### Risk 4: Memory Pressure (Many Threads × Large Vectors)
**Mitigation**: Limit max degree of parallelism; use object pooling for temporary arrays

---

## Conclusion

Parallelization can provide **4-8x speedup** for typical optimal control problems with minimal code changes. The highest ROI opportunities are:

1. **Numerical gradient computation** (embarrassingly parallel)
2. **Defect constraint evaluation** (embarrassingly parallel)
3. **Multiple shooting initialization** (naturally parallel)

These can be implemented incrementally with proper configuration APIs to ensure backward compatibility and allow users to control parallel behavior.

Next steps: Implement Phase 1 (foundation) to validate approach and measure actual performance gains.
