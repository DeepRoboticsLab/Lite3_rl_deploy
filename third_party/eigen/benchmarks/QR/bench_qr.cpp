// Benchmarks for QR decompositions.
//
// Tests HouseholderQR, ColPivHouseholderQR, FullPivHouseholderQR, and COD.
// Both square and tall-thin matrix shapes are tested.

#include <benchmark/benchmark.h>
#include <Eigen/QR>

using namespace Eigen;

template <typename QR>
EIGEN_DONT_INLINE void do_compute(QR& qr, const typename QR::MatrixType& A) {
  qr.compute(A);
}

// --- HouseholderQR ---

template <typename Scalar>
static void BM_HouseholderQR(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  HouseholderQR<Mat> qr(rows, cols);
  for (auto _ : state) {
    do_compute(qr, A);
    benchmark::DoNotOptimize(qr.matrixQR().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- ColPivHouseholderQR ---

template <typename Scalar>
static void BM_ColPivHouseholderQR(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  ColPivHouseholderQR<Mat> qr(rows, cols);
  for (auto _ : state) {
    do_compute(qr, A);
    benchmark::DoNotOptimize(qr.matrixQR().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- FullPivHouseholderQR ---

template <typename Scalar>
static void BM_FullPivHouseholderQR(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  FullPivHouseholderQR<Mat> qr(rows, cols);
  for (auto _ : state) {
    do_compute(qr, A);
    benchmark::DoNotOptimize(qr.matrixQR().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- CompleteOrthogonalDecomposition (COD) ---

template <typename Scalar>
static void BM_COD(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(rows, cols);
  CompleteOrthogonalDecomposition<Mat> cod(rows, cols);
  for (auto _ : state) {
    do_compute(cod, A);
    benchmark::DoNotOptimize(cod.matrixQTZ().data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- QR solve ---

template <typename Scalar>
static void BM_HouseholderQR_Solve(benchmark::State& state) {
  const Index rows = state.range(0);
  const Index cols = state.range(1);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  using Vec = Matrix<Scalar, Dynamic, 1>;
  Mat A = Mat::Random(rows, cols);
  Vec b = Vec::Random(rows);
  HouseholderQR<Mat> qr(A);
  Vec x(cols);
  for (auto _ : state) {
    x = qr.solve(b);
    benchmark::DoNotOptimize(x.data());
  }
  state.SetItemsProcessed(state.iterations());
}

// --- Size configurations ---

static void QrSizes(::benchmark::Benchmark* b) {
  // Square
  for (int n : {32, 64, 128, 256, 512, 1024}) b->Args({n, n});
  // Tall-thin
  b->Args({1000, 32});
  b->Args({1000, 100});
  b->Args({10000, 32});
  b->Args({10000, 100});
}

// Register: float
BENCHMARK(BM_HouseholderQR<float>)->Apply(QrSizes)->Name("HouseholderQR_float");
BENCHMARK(BM_ColPivHouseholderQR<float>)->Apply(QrSizes)->Name("ColPivHouseholderQR_float");
BENCHMARK(BM_FullPivHouseholderQR<float>)->Apply(QrSizes)->Name("FullPivHouseholderQR_float");
BENCHMARK(BM_COD<float>)->Apply(QrSizes)->Name("COD_float");
BENCHMARK(BM_HouseholderQR_Solve<float>)->Apply(QrSizes)->Name("HouseholderQR_Solve_float");

// Register: double
BENCHMARK(BM_HouseholderQR<double>)->Apply(QrSizes)->Name("HouseholderQR_double");
BENCHMARK(BM_ColPivHouseholderQR<double>)->Apply(QrSizes)->Name("ColPivHouseholderQR_double");
BENCHMARK(BM_FullPivHouseholderQR<double>)->Apply(QrSizes)->Name("FullPivHouseholderQR_double");
BENCHMARK(BM_COD<double>)->Apply(QrSizes)->Name("COD_double");
BENCHMARK(BM_HouseholderQR_Solve<double>)->Apply(QrSizes)->Name("HouseholderQR_Solve_double");
