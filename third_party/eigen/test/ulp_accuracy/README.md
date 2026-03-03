# ULP Accuracy Measurement Tool

Standalone tool for measuring the accuracy of Eigen's vectorized math functions
in units of ULP (Unit in the Last Place). Compares Eigen's SIMD implementations
against either MPFR (128-bit high-precision reference) or the standard C++ math
library.

## Building

From the Eigen build directory:

```bash
cd build
cmake ..
cmake --build . --target ulp_accuracy
```

If MPFR and GMP are installed, the build automatically enables MPFR support
(`EIGEN_HAS_MPFR`). Without them, only `--ref=std` is available.

### Installing MPFR (Debian/Ubuntu)

```bash
sudo apt install libmpfr-dev libgmp-dev
```

## Usage

```
./test/ulp_accuracy [options]

Options:
  --func=NAME    Function to test (required unless --list)
  --lo=VAL       Start of range (default: -inf)
  --hi=VAL       End of range (default: +inf)
  --double       Test double precision (default: float)
  --step=EPS     Sampling step: advance by (1+EPS)*nextafter(x)
                 (default: 0 = exhaustive; useful for double, e.g. 1e-6)
  --threads=N    Number of threads (default: all cores)
  --batch=N      Batch size for Eigen eval (default: 4096)
  --ref=MODE     Reference: 'std' (default) or 'mpfr'
  --hist_width=N Histogram half-width in ULPs (default: 10)
  --list         List available functions
```

## Examples

List all supported functions:
```bash
./test/ulp_accuracy --list
```

Exhaustive float test of sin against std (tests all ~4.28 billion finite floats):
```bash
./test/ulp_accuracy --func=sin
```

Float test against MPFR (more accurate reference, but slower):
```bash
./test/ulp_accuracy --func=sin --ref=mpfr
```

Double precision test with geometric sampling (exhaustive is impractical for double):
```bash
./test/ulp_accuracy --func=exp --double --step=1e-6
```

Test a specific range:
```bash
./test/ulp_accuracy --func=sin --lo=0 --hi=6.2832
```

## Output

The tool prints:

- **Test configuration**: function, range, reference mode, thread count
- **Max |ULP error|**: worst-case absolute ULP error with the offending input value
- **Mean |ULP error|**: average absolute ULP error across all tested values
- **Signed ULP histogram**: distribution of signed errors showing bias direction

Example output:
```
Function: sin (float)
Range: [-inf, inf]
Representable values in range: 4278190082
Reference: MPFR (128-bit)
Threads: 32
Batch size: 4096

Results:
  Values tested: 4278190081
  Time: 529.04 seconds (8.1 Mvalues/s)
  Max |ULP error|: 2
    at x = -1.5413464e+38 (Eigen=-0.482218683, ref=-0.482218742)
  Mean |ULP error|: 0.0874

Signed ULP error histogram [-10, +10]:
  -2   :        51988 (  0.001%)
  -1   :    186805349 (  4.366%)
  0    :   3904475407 ( 91.265%)
  1    :    186805349 (  4.366%)
  2    :        51988 (  0.001%)
```

## How it works

1. **Range splitting**: The input range is divided evenly across threads by
   splitting the linear ULP space.

2. **Batched evaluation**: Each thread fills batches of input values, evaluates
   them through Eigen's vectorized path (using `Eigen::Array` operations), and
   computes reference values one at a time.

3. **ULP computation**: IEEE 754 bit patterns are mapped to a linear integer
   scale where adjacent representable values are adjacent integers. The signed
   ULP error is the difference between Eigen's result and the reference on this
   scale. Special cases (NaN, infinity mismatches) report infinite error.

4. **Result reduction**: Per-thread statistics (max error, mean error, histogram)
   are merged after all threads complete.

## Supported functions

| Category | Functions |
|----------|-----------|
| Trigonometric | sin, cos, tan, asin, acos, atan |
| Hyperbolic | sinh, cosh, tanh, asinh, acosh, atanh |
| Exponential/Log | exp, exp2, expm1, log, log1p, log10, log2 |
| Error/Gamma | erf, erfc, lgamma |
| Other | logistic, sqrt, cbrt, rsqrt |

## File organization

- `ulp_accuracy.cpp` — Main tool: ULP computation, worker threads, CLI, result printing
- `mpfr_reference.h` — MPFR reference function wrappers and scalar conversion helpers

## Performance tips

- Float exhaustive sweeps test ~4.28 billion values. With `--ref=std` this takes
  ~50 seconds per function; with `--ref=mpfr` it takes ~500 seconds (10x slower).
- For double precision, exhaustive testing is impractical. Use `--step=1e-6` to
  sample ~2.88 billion values geometrically.
- Thread count defaults to all available cores. MPFR is the bottleneck (single
  MPFR call per value per thread), so more cores help significantly.
