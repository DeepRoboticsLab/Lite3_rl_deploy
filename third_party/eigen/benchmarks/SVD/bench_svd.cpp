#include <benchmark/benchmark.h>
#include <Eigen/Dense>

using namespace Eigen;

// Benchmark JacobiSVD and BDCSVD for various scalar types, matrix shapes,
// and computation options.

// ---------- helpers ----------

template <typename Scalar>
using Mat = Matrix<Scalar, Dynamic, Dynamic>;

template <typename SVD>
EIGEN_DONT_INLINE void do_compute(SVD& svd, const typename SVD::MatrixType& A) {
  svd.compute(A);
}

// ---------- JacobiSVD ----------

template <typename Scalar, int Options>
static void BM_JacobiSVD(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  Mat<Scalar> A = Mat<Scalar>::Random(rows, cols);
  JacobiSVD<Mat<Scalar>, Options> svd(rows, cols);
  for (auto _ : state) {
    do_compute(svd, A);
    benchmark::DoNotOptimize(svd.singularValues().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// ---------- BDCSVD ----------

template <typename Scalar, int Options>
static void BM_BDCSVD(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  Mat<Scalar> A = Mat<Scalar>::Random(rows, cols);
  BDCSVD<Mat<Scalar>, Options> svd(rows, cols);
  for (auto _ : state) {
    do_compute(svd, A);
    benchmark::DoNotOptimize(svd.singularValues().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// ---------- Size configurations ----------

// Sizes suitable for JacobiSVD (O(n^2 p), expensive for large n).
static void JacobiSizes(::benchmark::Benchmark* b) {
  // Square
  for (int s : {4, 8, 16, 32, 64, 128, 256, 512}) b->Args({s, s});
  // Tall-skinny
  b->Args({100, 4});
  b->Args({1000, 4});
  b->Args({1000, 10});
}

// Sizes suitable for BDCSVD (divide-and-conquer, faster for large matrices).
static void BDCSizes(::benchmark::Benchmark* b) {
  // Square
  for (int s : {4, 8, 16, 32, 64, 128, 256, 512, 1024}) b->Args({s, s});
  // Tall-skinny (triggers R-bidiagonalization when aspect ratio > 4)
  b->Args({100, 4});
  b->Args({1000, 4});
  b->Args({1000, 10});
  b->Args({1000, 100});
  b->Args({10000, 10});
  b->Args({10000, 100});
}

// ---------- Register benchmarks ----------

// JacobiSVD — float
BENCHMARK(BM_JacobiSVD<float, ComputeThinU | ComputeThinV>)->Apply(JacobiSizes)->Name("JacobiSVD_float_ThinUV");
BENCHMARK(BM_JacobiSVD<float, 0>)->Apply(JacobiSizes)->Name("JacobiSVD_float_ValuesOnly");

// JacobiSVD — double
BENCHMARK(BM_JacobiSVD<double, ComputeThinU | ComputeThinV>)->Apply(JacobiSizes)->Name("JacobiSVD_double_ThinUV");
BENCHMARK(BM_JacobiSVD<double, 0>)->Apply(JacobiSizes)->Name("JacobiSVD_double_ValuesOnly");

// BDCSVD — float
BENCHMARK(BM_BDCSVD<float, ComputeThinU | ComputeThinV>)->Apply(BDCSizes)->Name("BDCSVD_float_ThinUV");
BENCHMARK(BM_BDCSVD<float, 0>)->Apply(BDCSizes)->Name("BDCSVD_float_ValuesOnly");

// BDCSVD — double
BENCHMARK(BM_BDCSVD<double, ComputeThinU | ComputeThinV>)->Apply(BDCSizes)->Name("BDCSVD_double_ThinUV");
BENCHMARK(BM_BDCSVD<double, 0>)->Apply(BDCSizes)->Name("BDCSVD_double_ValuesOnly");

// JacobiSVD — QR preconditioner comparison (double, 64x64, ThinUV)
BENCHMARK(BM_JacobiSVD<double, ComputeThinU | ComputeThinV | ColPivHouseholderQRPreconditioner>)
    ->Args({64, 64})
    ->Args({1000, 10})
    ->Name("JacobiSVD_double_ColPivQR");
BENCHMARK(BM_JacobiSVD<double, ComputeThinU | ComputeThinV | HouseholderQRPreconditioner>)
    ->Args({64, 64})
    ->Args({1000, 10})
    ->Name("JacobiSVD_double_HouseholderQR");
BENCHMARK(BM_JacobiSVD<double, ComputeFullU | ComputeFullV | FullPivHouseholderQRPreconditioner>)
    ->Args({64, 64})
    ->Name("JacobiSVD_double_FullPivQR");
