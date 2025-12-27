# Optimal Library - Current Limitations

Based on analysis of the codebase, here are the **current limitations** of what code can be used with Optimal's automatic differentiation:

## ✅ Supported Features

### Data Types
- ✅ `double` (primary type)
- ✅ `float`
- ✅ `int` (as constants)

### Operators
- ✅ Binary: `+`, `-`, `*`, `/`
- ✅ Comparison: `>`, `<`, `>=`, `<=`, `==`, `!=`
- ✅ Unary: `-`, `+`
- ✅ Power operator (via `Math.Pow`)

### Math Functions (System.Math)
- ✅ `Sqrt(x)` - Square root
- ✅ `Sin(x)`, `Cos(x)`, `Tan(x)` - Trigonometric
- ✅ `Exp(x)` - Exponential
- ✅ `Log(x)` - Natural logarithm
- ✅ `Pow(x, n)` - Power (constant exponent preferred)
- ✅ `Abs(x)` - Absolute value
- ✅ `Atan2(y, x)` - Two-argument arctangent

### Control Flow
- ✅ `if/else` statements
- ✅ Ternary operator `? :`
- ✅ `for` loops

### Function Features
- ✅ Expression-bodied functions
- ✅ Statement-bodied functions
- ✅ Local variables
- ✅ Multiple parameters
- ✅ User-defined nested functions (same class)

## ❌ Current Limitations

### 1. **Statement Types**
```csharp
// ❌ NOT SUPPORTED
while (condition) { }          // while loops
do { } while (condition);      // do-while loops
foreach (var x in collection)  // foreach loops
switch (x) { }                 // switch statements
break;                         // break statements
continue;                      // continue statements
goto label;                    // goto statements
try { } catch { }              // exception handling
```

### 2. **Expression Types**
```csharp
// ❌ NOT SUPPORTED
var lambda = (x) => x * x;     // Lambda expressions
var func = delegate(x) { };    // Anonymous methods
x?.Property                     // Null-conditional operators
x ?? y                          // Null-coalescing
obj.Method()                    // Instance method calls (except Math)
array[i]                        // Array indexing
list[i]                         // Collection indexing
new Type()                      // Object creation
(int)x                          // Casts (except identity casts for differentiation)
x is Type                       // Type checking
x as Type                       // Type casting with as
await Task                      // Async/await
```

### 3. **Math Functions Not Implemented**
```csharp
// ❌ NOT SUPPORTED (but could be added)
Math.Asin(x)                   // Arcsine
Math.Acos(x)                   // Arccosine
Math.Atan(x)                   // Arctangent (single arg)
Math.Sinh(x)                   // Hyperbolic sine
Math.Cosh(x)                   // Hyperbolic cosine
Math.Tanh(x)                   // Hyperbolic tangent
Math.Floor(x)                  // Floor (non-differentiable)
Math.Ceiling(x)                // Ceiling (non-differentiable)
Math.Round(x)                  // Round (non-differentiable)
Math.Min(x, y)                 // Minimum (non-smooth)
Math.Max(x, y)                 // Maximum (non-smooth)
Math.Sign(x)                   // Sign (discontinuous)
Math.Log10(x)                  // Base-10 logarithm
Math.Log(x, b)                 // Logarithm with base
```

### 4. **Complex Function Bodies**
```csharp
// ❌ NOT SUPPORTED for nested functions
public static double ComplexNested(double x)
{
    // Multiple statements with control flow in nested calls
    if (x > 0)
        return Helper(x);
    else
        return Helper(-x);
}

// Only simple expression bodies or single-statement bodies work
// for nested function inlining
```

### 5. **Variable Exponents in Power**
```csharp
// ⚠️ LIMITED SUPPORT
Math.Pow(x, y)  // Variable exponent - limited support
// Works but differentiation is more complex
// Better to use constant exponents when possible
```

### 6. **Data Structures**
```csharp
// ❌ NOT SUPPORTED
double[] array = new double[10];    // Arrays
List<double> list = new();          // Collections
var tuple = (1.0, 2.0);            // Tuples (except return types)
class MyClass { }                   // Custom classes as params
```

### 7. **Advanced C# Features**
```csharp
// ❌ NOT SUPPORTED
ref double x;                       // Ref variables
out double x;                       // Out parameters
in double x;                        // In parameters
params double[] x;                  // Params arrays
default(double)                     // Default expressions
nameof(x)                          // Nameof expressions
typeof(double)                     // Typeof expressions
x => x * x                         // Expression trees
yield return x;                    // Iterators
```

### 8. **Method Constraints**
```csharp
// ❌ MUST BE:
[OptimalCode]
public static class MyClass  // ✅ static class
{
    // ✅ public, static methods only
    public static double F(double x) => x * x;
    
    // ❌ NOT SUPPORTED:
    private static double G(double x) => x;  // private
    public double H(double x) => x;          // non-static
    internal static double I(double x) => x; // internal
}
```

### 9. **Recursive Functions**
```csharp
// ❌ NOT SUPPORTED
public static double Factorial(double n)
{
    if (n <= 1) return 1;
    return n * Factorial(n - 1);  // Infinite inlining!
}
```

### 10. **Cross-Class Function Calls**
```csharp
[OptimalCode]
public static class ClassA
{
    public static double F(double x) => x * x;
}

[OptimalCode]
public static class ClassB
{
    // ❌ NOT SUPPORTED
    public static double G(double x) => ClassA.F(x);
    // Can't call functions from another [OptimalCode] class
}
```

## 🔄 Reverse Mode Limitations

Reverse mode automatic differentiation has **not been updated** for nested functions yet:
- ✅ Works for non-nested functions
- ❌ Does not support nested user-defined functions
- ⚠️ Will be added in future updates

## 📊 Summary

**Best suited for:**
- Mathematical computations with basic arithmetic
- Smooth, continuous functions
- Physics simulations
- Optimization algorithms
- Gradient-based machine learning primitives

**Not suited for:**
- General-purpose C# programming
- Complex control flow with many branches
- Data structure manipulation
- Object-oriented code
- Recursive algorithms
- Non-smooth functions (min, max, abs, floor, etc.)

## 💡 Workarounds

Some limitations can be worked around:

```csharp
// Instead of Math.Min (non-smooth):
public static double SmoothMin(double a, double b, double k)
{
    // Smooth approximation using exponentials
    var h = Math.Max(k - Math.Abs(a - b), 0.0) / k;
    return Math.Min(a, b) - h * h * k * 0.25;
}

// Instead of if/else discontinuities:
public static double SmoothStep(double x)
{
    // Use smooth functions instead
    return 1.0 / (1.0 + Math.Exp(-x));  // Sigmoid
}

// Instead of array indexing:
// Pass individual values as parameters
public static double ProcessThree(double x1, double x2, double x3)
{
    return x1 + x2 * x3;
}
```

## 🚀 Future Enhancements

Easy to add:
- More Math functions (Asin, Acos, Atan, Sinh, Cosh, Tanh)
- While/do-while loops
- Reverse mode for nested functions

Harder to add:
- Array/collection support
- Recursive functions (requires memoization)
- Cross-class function calls
- Non-smooth function approximations
- Higher-order differentiation
