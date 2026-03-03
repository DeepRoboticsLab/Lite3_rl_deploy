// Standalone tool to measure ULP accuracy of Eigen's vectorized math functions
// against either MPFR (high-precision reference) or std C++ math functions.
//
// See README.md in this directory for full documentation.
//
// Usage:
//   ./ulp_accuracy --func=sin --lo=0 --hi=6.2832 --threads=16
//   ./ulp_accuracy --func=exp --threads=16
//   ./ulp_accuracy --func=sin --ref=mpfr
//   ./ulp_accuracy --func=sin --double --step=1e-6
//   ./ulp_accuracy --list
//
// Build:
//   cd build && cmake .. && make ulp_accuracy

#include <Eigen/Core>
#include <unsupported/Eigen/SpecialFunctions>

#ifdef EIGEN_HAS_MPFR
#include <mpfr.h>
#endif

#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "mpfr_reference.h"

// ============================================================================
// ULP distance computation
// ============================================================================

// Maps IEEE 754 bits to a linear integer scale where adjacent representable
// values are adjacent integers. The mapping is strictly monotonic:
//   -inf -> most negative, -0.0 -> -1, +0.0 -> 0, +inf -> most positive.
static inline int64_t scalar_to_linear(float x) {
  int32_t bits;
  std::memcpy(&bits, &x, sizeof(bits));
  if (bits < 0) {
    bits = static_cast<int32_t>(INT32_MIN) - bits - 1;
  }
  return static_cast<int64_t>(bits);
}

static inline int64_t scalar_to_linear(double x) {
  int64_t bits;
  std::memcpy(&bits, &x, sizeof(bits));
  if (bits < 0) {
    bits = static_cast<int64_t>(INT64_MIN) - bits - 1;
  }
  return bits;
}

// Returns (eigen_val - ref_val) in ULP space.
// Positive means Eigen overestimates, negative means it underestimates.
// Returns INT64_MAX for incomparable values (NaN vs number, inf mismatch).
template <typename Scalar>
static inline int64_t signed_ulp_error(Scalar eigen_val, Scalar ref_val) {
  if (eigen_val == ref_val) return 0;  // also handles -0.0 == +0.0
  bool e_nan = std::isnan(eigen_val), r_nan = std::isnan(ref_val);
  if (e_nan && r_nan) return 0;
  if (e_nan || r_nan) return INT64_MAX;
  if (std::isinf(eigen_val) || std::isinf(ref_val)) return INT64_MAX;
  int64_t a = scalar_to_linear(eigen_val);
  int64_t b = scalar_to_linear(ref_val);
  // Overflow check for a - b.
  if (b > 0 && a < INT64_MIN + b) return INT64_MAX;
  if (b < 0 && a > INT64_MAX + b) return INT64_MAX;
  return a - b;
}

// ============================================================================
// Per-thread accumulator with signed ULP histogram
// ============================================================================

template <typename Scalar>
struct alignas(128) ThreadResult {
  int64_t max_abs_ulp = 0;
  Scalar max_ulp_at = Scalar(0);
  Scalar max_ulp_eigen = Scalar(0);
  Scalar max_ulp_ref = Scalar(0);
  double abs_ulp_sum = 0.0;
  uint64_t count = 0;

  // Signed histogram: bins for errors in [-hist_width, +hist_width],
  // plus two overflow bins for < -hist_width and > +hist_width.
  // Layout: [<-W] [-W] [-W+1] ... [0] ... [W-1] [W] [>W]
  // Total bins = 2*hist_width + 3
  int hist_width = 0;
  std::vector<uint64_t> hist;

  void init(int w) {
    hist_width = w;
    hist.assign(2 * w + 3, 0);
  }

  void record(int64_t signed_err, Scalar x, Scalar eigen_val, Scalar ref_val) {
    int64_t abs_err = signed_err < 0 ? -signed_err : signed_err;
    if (signed_err == INT64_MAX) abs_err = INT64_MAX;

    if (abs_err > max_abs_ulp) {
      max_abs_ulp = abs_err;
      max_ulp_at = x;
      max_ulp_eigen = eigen_val;
      max_ulp_ref = ref_val;
    }
    if (abs_err != INT64_MAX) {
      abs_ulp_sum += static_cast<double>(abs_err);
    }
    count++;

    // Histogram bin.
    int bin;
    if (signed_err == INT64_MAX || signed_err > hist_width) {
      bin = 2 * hist_width + 2;  // overflow high
    } else if (signed_err < -hist_width) {
      bin = 0;  // overflow low
    } else {
      bin = static_cast<int>(signed_err) + hist_width + 1;
    }
    hist[bin]++;
  }
};

// ============================================================================
// Function registry
// ============================================================================

template <typename Scalar>
struct FuncEntry {
  using ArrayType = Eigen::Array<Scalar, Eigen::Dynamic, 1>;
  using EigenEval = std::function<void(Eigen::Ref<ArrayType>, const Eigen::Ref<const ArrayType>&)>;
  using StdEval = std::function<Scalar(Scalar)>;

#ifdef EIGEN_HAS_MPFR
  using MpfrEval = std::function<int(mpfr_t, const mpfr_t, mpfr_rnd_t)>;
#endif

  std::string name;
  EigenEval eigen_eval;
  StdEval std_eval;
#ifdef EIGEN_HAS_MPFR
  MpfrEval mpfr_eval;
#endif
  Scalar default_lo;
  Scalar default_hi;
};

// std::logistic is not part of the C++ standard library.
template <typename Scalar>
static Scalar std_logistic(Scalar x) {
  if (x >= 0) {
    Scalar e = std::exp(-x);
    return Scalar(1) / (Scalar(1) + e);
  } else {
    Scalar e = std::exp(x);
    return e / (Scalar(1) + e);
  }
}

template <typename Scalar>
static std::vector<FuncEntry<Scalar>> build_func_table() {
  using ArrayType = Eigen::Array<Scalar, Eigen::Dynamic, 1>;
  std::vector<FuncEntry<Scalar>> table;

#ifdef EIGEN_HAS_MPFR
#define ADD_FUNC(fname, eigen_expr, std_expr, mpfr_fn, lo, hi)                                                        \
  table.push_back({#fname, [](Eigen::Ref<ArrayType> out, const Eigen::Ref<const ArrayType>& a) { out = eigen_expr; }, \
                   [](Scalar x) -> Scalar { return std_expr; },                                                       \
                   [](mpfr_t r, const mpfr_t o, mpfr_rnd_t d) { return mpfr_fn(r, o, d); }, lo, hi})
#else
#define ADD_FUNC(fname, eigen_expr, std_expr, mpfr_fn, lo, hi)                                                        \
  table.push_back({#fname, [](Eigen::Ref<ArrayType> out, const Eigen::Ref<const ArrayType>& a) { out = eigen_expr; }, \
                   [](Scalar x) -> Scalar { return std_expr; }, lo, hi})
#endif

  constexpr Scalar kInf = std::numeric_limits<Scalar>::infinity();

  // Trigonometric
  // clang-format off
  ADD_FUNC(sin,   a.sin(),   std::sin(x),   mpfr_sin,   -kInf, kInf);
  ADD_FUNC(cos,   a.cos(),   std::cos(x),   mpfr_cos,   -kInf, kInf);
  ADD_FUNC(tan,   a.tan(),   std::tan(x),   mpfr_tan,   -kInf, kInf);
  ADD_FUNC(asin,  a.asin(),  std::asin(x),  mpfr_asin,  -kInf, kInf);
  ADD_FUNC(acos,  a.acos(),  std::acos(x),  mpfr_acos,  -kInf, kInf);
  ADD_FUNC(atan,  a.atan(),  std::atan(x),  mpfr_atan,  -kInf, kInf);

  // Hyperbolic
  ADD_FUNC(sinh,  a.sinh(),  std::sinh(x),  mpfr_sinh,  -kInf, kInf);
  ADD_FUNC(cosh,  a.cosh(),  std::cosh(x),  mpfr_cosh,  -kInf, kInf);
  ADD_FUNC(tanh,  a.tanh(),  std::tanh(x),  mpfr_tanh,  -kInf, kInf);
  ADD_FUNC(asinh, a.asinh(), std::asinh(x), mpfr_asinh, -kInf, kInf);
  ADD_FUNC(acosh, a.acosh(), std::acosh(x), mpfr_acosh, -kInf, kInf);
  ADD_FUNC(atanh, a.atanh(), std::atanh(x), mpfr_atanh, -kInf, kInf);

  // Exponential / Logarithmic
  ADD_FUNC(exp,   a.exp(),     std::exp(x),    mpfr_exp,       -kInf, kInf);
  ADD_FUNC(exp2,  a.exp2(),    std::exp2(x),   mpfr_exp2_wrap, -kInf, kInf);
  ADD_FUNC(expm1, a.expm1(),   std::expm1(x),  mpfr_expm1,     -kInf, kInf);
  ADD_FUNC(log,   a.log(),     std::log(x),    mpfr_log,       -kInf, kInf);
  ADD_FUNC(log1p, a.log1p(),   std::log1p(x),  mpfr_log1p,     -kInf, kInf);
  ADD_FUNC(log10, a.log10(),   std::log10(x),  mpfr_log10,     -kInf, kInf);
  ADD_FUNC(log2,  a.log2(),    std::log2(x),   mpfr_log2_wrap, -kInf, kInf);

  // Error / Gamma
  ADD_FUNC(erf,    a.erf(),    std::erf(x),    mpfr_erf,     -kInf, kInf);
  ADD_FUNC(erfc,   a.erfc(),   std::erfc(x),   mpfr_erfc,    -kInf, kInf);
  ADD_FUNC(lgamma, a.lgamma(), std::lgamma(x), mpfr_lngamma, -kInf, kInf);

  // Other
  ADD_FUNC(logistic, a.logistic(), std_logistic(x), mpfr_logistic, -kInf, kInf);
  ADD_FUNC(sqrt,  a.sqrt(),  std::sqrt(x),            mpfr_sqrt,  -kInf, kInf);
  ADD_FUNC(cbrt,  a.cbrt(),  std::cbrt(x),            mpfr_cbrt,  -kInf, kInf);
  ADD_FUNC(rsqrt, a.rsqrt(), Scalar(1)/std::sqrt(x),  mpfr_rsqrt, -kInf, kInf);
  // clang-format on

#undef ADD_FUNC
  return table;
}

// ============================================================================
// Range iteration helpers
// ============================================================================

// Advances x toward +inf by at least 1 ULP. When step_eps > 0, additionally
// jumps by a relative factor of (1 + step_eps) to sample the range sparsely.
template <typename Scalar>
static inline Scalar advance_by_step(Scalar x, double step_eps) {
  Scalar next = std::nextafter(x, std::numeric_limits<Scalar>::infinity());
  if (step_eps > 0.0 && std::isfinite(next)) {
    // Try to jump further by a relative amount.
    Scalar jumped = next > 0 ? next * static_cast<Scalar>(1.0 + step_eps) : next / static_cast<Scalar>(1.0 + step_eps);
    // Use the jump only if it actually advances further (handles denormal stalling).
    if (jumped > next) next = jumped;
  }
  return next;
}

// Counts the number of representable scalars in [lo, hi].
template <typename Scalar>
static uint64_t count_scalars_in_range(Scalar lo, Scalar hi) {
  if (lo > hi) return 0;
  uint64_t lo_u = static_cast<uint64_t>(scalar_to_linear(lo));
  uint64_t hi_u = static_cast<uint64_t>(scalar_to_linear(hi));
  uint64_t diff = hi_u - lo_u;
  return diff == UINT64_MAX ? UINT64_MAX : diff + 1;
}

// Advances a scalar by n ULPs in the linear representation.
static float advance_scalar(float x, uint64_t n) {
  int64_t lin = scalar_to_linear(x);
  lin += static_cast<int64_t>(n);
  int32_t ibits;
  if (lin < 0) {
    ibits = static_cast<int32_t>(INT32_MIN) - static_cast<int32_t>(lin) - 1;
  } else {
    ibits = static_cast<int32_t>(lin);
  }
  float result;
  std::memcpy(&result, &ibits, sizeof(result));
  return result;
}

static double advance_scalar(double x, uint64_t n) {
  int64_t lin = scalar_to_linear(x);
  lin += static_cast<int64_t>(n);
  int64_t ibits;
  if (lin < 0) {
    ibits = static_cast<int64_t>(INT64_MIN) - lin - 1;
  } else {
    ibits = lin;
  }
  double result;
  std::memcpy(&result, &ibits, sizeof(result));
  return result;
}

// ============================================================================
// Worker thread: evaluates Eigen and reference over a subrange
// ============================================================================

template <typename Scalar>
static void worker(const FuncEntry<Scalar>& func, Scalar lo, Scalar hi, int batch_size, bool use_mpfr, double step_eps,
                   ThreadResult<Scalar>& result) {
  using ArrayType = Eigen::Array<Scalar, Eigen::Dynamic, 1>;
  ArrayType input(batch_size);
  ArrayType eigen_out(batch_size);
  std::vector<Scalar> ref_out(batch_size);

#ifdef EIGEN_HAS_MPFR
  mpfr_t mp_in, mp_out;
  if (use_mpfr) {
    mpfr_init2(mp_in, 128);
    mpfr_init2(mp_out, 128);
  }
#else
  (void)use_mpfr;
#endif

  auto process_batch = [&](int n, const ArrayType& in, const ArrayType& eig) {
    for (int i = 0; i < n; i++) {
#ifdef EIGEN_HAS_MPFR
      if (use_mpfr) {
        mpfr_set_scalar<Scalar>(mp_in, in[i], MPFR_RNDN);
        func.mpfr_eval(mp_out, mp_in, MPFR_RNDN);
        ref_out[i] = mpfr_get_scalar<Scalar>(mp_out, MPFR_RNDN);
      } else
#endif
      {
        ref_out[i] = func.std_eval(in[i]);
      }
    }
    for (int i = 0; i < n; i++) {
      int64_t err = signed_ulp_error(eig[i], ref_out[i]);
      result.record(err, in[i], eig[i], ref_out[i]);
    }
  };

  int idx = 0;
  Scalar x = lo;
  for (;;) {
    input[idx] = x;
    idx++;

    if (idx == batch_size) {
      func.eigen_eval(eigen_out, input);
      process_batch(batch_size, input, eigen_out);
      idx = 0;
    }

    if (x >= hi) break;
    Scalar next = advance_by_step(x, step_eps);
    x = (next > hi) ? hi : next;
  }

  // Process remaining partial batch.
  if (idx > 0) {
    auto partial_in = input.head(idx);
    auto partial_eigen = eigen_out.head(idx);
    func.eigen_eval(partial_eigen, partial_in);
    process_batch(idx, input, eigen_out);
  }

#ifdef EIGEN_HAS_MPFR
  if (use_mpfr) {
    mpfr_clear(mp_in);
    mpfr_clear(mp_out);
  }
#endif
}

// ============================================================================
// Test driver: splits range across threads and prints results
// ============================================================================

struct Options {
  std::string func_name;
  double lo = std::numeric_limits<double>::quiet_NaN();
  double hi = std::numeric_limits<double>::quiet_NaN();
  int num_threads;
  int batch_size = 4096;
  int hist_width = 10;
  bool use_mpfr = false;
  bool use_double = false;
  double step_eps = 0.0;
  bool list_funcs = false;
};

template <typename Scalar>
static int run_test(const Options& opts) {
  const int kDigits = std::is_same<Scalar, float>::value ? 9 : 17;
  const char* kTypeName = std::is_same<Scalar, float>::value ? "float" : "double";

  auto table = build_func_table<Scalar>();

  if (opts.list_funcs) {
    std::printf("Available functions:\n");
    for (const auto& f : table) {
      std::printf("  %s\n", f.name.c_str());
    }
    return 0;
  }

  // Look up the requested function.
  const FuncEntry<Scalar>* entry = nullptr;
  for (const auto& f : table) {
    if (f.name == opts.func_name) {
      entry = &f;
      break;
    }
  }
  if (!entry) {
    std::fprintf(stderr, "Error: unknown function '%s' (use --list to see available functions)\n",
                 opts.func_name.c_str());
    return 1;
  }

  Scalar lo = std::isnan(opts.lo) ? entry->default_lo : static_cast<Scalar>(opts.lo);
  Scalar hi = std::isnan(opts.hi) ? entry->default_hi : static_cast<Scalar>(opts.hi);
  uint64_t total_scalars = count_scalars_in_range(lo, hi);
  int num_threads = opts.num_threads;

  // Print test configuration.
  std::printf("Function: %s (%s)\n", opts.func_name.c_str(), kTypeName);
  std::printf("Range: [%.*g, %.*g]\n", kDigits, double(lo), kDigits, double(hi));
  if (opts.step_eps > 0.0) {
    std::printf("Sampling step: (1 + %g) * nextafter(x)\n", opts.step_eps);
  } else {
    std::printf("Representable values in range: %lu\n", static_cast<unsigned long>(total_scalars));
  }
  std::printf("Reference: %s\n", opts.use_mpfr ? "MPFR (128-bit)" : "std C++ math");
  std::printf("Threads: %d\n", num_threads);
  std::printf("Batch size: %d\n", opts.batch_size);
  std::printf("\n");
  std::fflush(stdout);

  // Split range across threads.
  if (total_scalars > 0 && static_cast<uint64_t>(num_threads) > total_scalars) {
    num_threads = static_cast<int>(total_scalars);
  }
  if (num_threads < 1) num_threads = 1;

  // Heap-allocate each ThreadResult separately to avoid false sharing.
  std::vector<std::unique_ptr<ThreadResult<Scalar>>> results;
  results.reserve(num_threads);
  for (int t = 0; t < num_threads; t++) {
    results.push_back(std::make_unique<ThreadResult<Scalar>>());
    results.back()->init(opts.hist_width);
  }

  std::vector<std::thread> threads;
  uint64_t scalars_per_thread = total_scalars / num_threads;
  Scalar chunk_lo = lo;

  auto start_time = std::chrono::steady_clock::now();

  for (int t = 0; t < num_threads; t++) {
    Scalar chunk_hi;
    if (t == num_threads - 1) {
      chunk_hi = hi;
    } else {
      chunk_hi = advance_scalar(chunk_lo, scalars_per_thread - 1);
    }
    threads.emplace_back(worker<Scalar>, std::cref(*entry), chunk_lo, chunk_hi, opts.batch_size, opts.use_mpfr,
                         opts.step_eps, std::ref(*results[t]));
    chunk_lo = std::nextafter(chunk_hi, std::numeric_limits<Scalar>::infinity());
  }

  for (auto& t : threads) t.join();
  auto end_time = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(end_time - start_time).count();

  // Reduce per-thread results.
  ThreadResult<Scalar> global;
  global.init(opts.hist_width);
  for (int t = 0; t < num_threads; t++) {
    const auto& r = *results[t];
    if (r.max_abs_ulp > global.max_abs_ulp) {
      global.max_abs_ulp = r.max_abs_ulp;
      global.max_ulp_at = r.max_ulp_at;
      global.max_ulp_eigen = r.max_ulp_eigen;
      global.max_ulp_ref = r.max_ulp_ref;
    }
    global.abs_ulp_sum += r.abs_ulp_sum;
    global.count += r.count;
    for (size_t b = 0; b < global.hist.size(); b++) {
      global.hist[b] += r.hist[b];
    }
  }

  double mean_ulp = global.count > 0 ? global.abs_ulp_sum / global.count : 0.0;

  // Print results.
  std::printf("Results:\n");
  std::printf("  Values tested: %lu\n", static_cast<unsigned long>(global.count));
  std::printf("  Time: %.2f seconds (%.1f Mvalues/s)\n", elapsed, global.count / elapsed / 1e6);
  if (global.max_abs_ulp == INT64_MAX) {
    std::printf("  Max |ULP error|: inf\n");
  } else {
    std::printf("  Max |ULP error|: %ld\n", static_cast<long>(global.max_abs_ulp));
  }
  std::printf("    at x = %.*g (Eigen=%.*g, ref=%.*g)\n", kDigits, double(global.max_ulp_at), kDigits,
              double(global.max_ulp_eigen), kDigits, double(global.max_ulp_ref));
  std::printf("  Mean |ULP error|: %.4f\n", mean_ulp);
  std::printf("\n");

  // Print signed error histogram.
  std::printf("Signed ULP error histogram [-%d, +%d]:\n", opts.hist_width, opts.hist_width);
  int nbins = 2 * opts.hist_width + 3;
  for (int b = 0; b < nbins; b++) {
    if (global.hist[b] == 0) continue;
    double pct = 100.0 * global.hist[b] / global.count;
    if (b == 0) {
      std::printf("  <%-4d: %12lu (%7.3f%%)\n", -opts.hist_width, static_cast<unsigned long>(global.hist[b]), pct);
    } else if (b == nbins - 1) {
      std::printf("  >%-4d: %12lu (%7.3f%%)\n", opts.hist_width, static_cast<unsigned long>(global.hist[b]), pct);
    } else {
      int err = b - opts.hist_width - 1;
      std::printf("  %-5d: %12lu (%7.3f%%)\n", err, static_cast<unsigned long>(global.hist[b]), pct);
    }
  }

  return 0;
}

// ============================================================================
// Command-line parsing
// ============================================================================

static void print_usage() {
  std::printf(
      "Usage: ulp_accuracy [options]\n"
      "  --func=NAME    Function to test (required unless --list)\n"
      "  --lo=VAL       Start of range (default: -inf)\n"
      "  --hi=VAL       End of range (default: +inf)\n"
      "  --double       Test double precision (default: float)\n"
      "  --step=EPS     Sampling step: advance by (1+EPS)*nextafter(x)\n"
      "                 (default: 0 = exhaustive; useful for double, e.g. 1e-6)\n"
      "  --threads=N    Number of threads (default: all cores)\n"
      "  --batch=N      Batch size for Eigen eval (default: 4096)\n"
      "  --ref=MODE     Reference: 'std' (default) or 'mpfr'\n"
      "  --hist_width=N Histogram half-width in ULPs (default: 10)\n"
      "  --list         List available functions\n");
}

int main(int argc, char* argv[]) {
  Options opts;
  opts.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  if (opts.num_threads == 0) opts.num_threads = 4;
  std::string ref_mode;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg.substr(0, 7) == "--func=") {
      opts.func_name = arg.substr(7);
    } else if (arg.substr(0, 5) == "--lo=") {
      std::string val = arg.substr(5);
      if (val == "inf")
        opts.lo = std::numeric_limits<double>::infinity();
      else if (val == "-inf")
        opts.lo = -std::numeric_limits<double>::infinity();
      else
        opts.lo = std::stod(val);
    } else if (arg.substr(0, 5) == "--hi=") {
      std::string val = arg.substr(5);
      if (val == "inf")
        opts.hi = std::numeric_limits<double>::infinity();
      else if (val == "-inf")
        opts.hi = -std::numeric_limits<double>::infinity();
      else
        opts.hi = std::stod(val);
    } else if (arg.substr(0, 10) == "--threads=") {
      opts.num_threads = std::stoi(arg.substr(10));
    } else if (arg.substr(0, 8) == "--batch=") {
      opts.batch_size = std::stoi(arg.substr(8));
    } else if (arg.substr(0, 6) == "--ref=") {
      ref_mode = arg.substr(6);
    } else if (arg.substr(0, 13) == "--hist_width=") {
      opts.hist_width = std::stoi(arg.substr(13));
    } else if (arg.substr(0, 7) == "--step=") {
      opts.step_eps = std::stod(arg.substr(7));
    } else if (arg == "--double") {
      opts.use_double = true;
    } else if (arg == "--list") {
      opts.list_funcs = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else {
      std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
      print_usage();
      return 1;
    }
  }

  // Determine reference mode (default: std).
  if (ref_mode.empty() || ref_mode == "std") {
    opts.use_mpfr = false;
  } else if (ref_mode == "mpfr") {
#ifdef EIGEN_HAS_MPFR
    opts.use_mpfr = true;
#else
    std::fprintf(stderr, "Error: MPFR support not compiled in. Use --ref=std or rebuild with MPFR.\n");
    return 1;
#endif
  } else {
    std::fprintf(stderr, "Error: --ref must be 'std' or 'mpfr'\n");
    return 1;
  }

  if (!opts.list_funcs && opts.func_name.empty()) {
    std::fprintf(stderr, "Error: --func=NAME is required (use --list to see available functions)\n");
    return 1;
  }

  if (opts.use_double) {
    return run_test<double>(opts);
  } else {
    return run_test<float>(opts);
  }
}
