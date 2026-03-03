#include <benchmark/benchmark.h>
#include <Eigen/Dense>

using namespace Eigen;

typedef float Scalar;
typedef Matrix<Scalar, Dynamic, Dynamic> Mat;

template <typename Solver>
EIGEN_DONT_INLINE void compute_norm_equation(Solver& solver, const Mat& A) {
  if (A.rows() != A.cols())
    solver.compute(A.transpose() * A);
  else
    solver.compute(A);
}

template <typename Solver>
EIGEN_DONT_INLINE void compute(Solver& solver, const Mat& A) {
  solver.compute(A);
}

static void BM_LLT(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  int size = cols;
  Mat A(rows, cols);
  A.setRandom();
  if (rows == cols) A = A * A.adjoint();
  LLT<Mat> solver(size);
  for (auto _ : state) {
    compute_norm_equation(solver, A);
    benchmark::DoNotOptimize(solver.matrixLLT().data());
  }
}

static void BM_LDLT(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  int size = cols;
  Mat A(rows, cols);
  A.setRandom();
  if (rows == cols) A = A * A.adjoint();
  LDLT<Mat> solver(size);
  for (auto _ : state) {
    compute_norm_equation(solver, A);
    benchmark::DoNotOptimize(solver.matrixLDLT().data());
  }
}

static void BM_PartialPivLU(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  int size = cols;
  Mat A(rows, cols);
  A.setRandom();
  if (rows == cols) A = A * A.adjoint();
  PartialPivLU<Mat> solver(size);
  for (auto _ : state) {
    compute_norm_equation(solver, A);
    benchmark::DoNotOptimize(solver.matrixLU().data());
  }
}

static void BM_FullPivLU(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  int size = cols;
  Mat A(rows, cols);
  A.setRandom();
  if (rows == cols) A = A * A.adjoint();
  FullPivLU<Mat> solver(size, size);
  for (auto _ : state) {
    compute_norm_equation(solver, A);
    benchmark::DoNotOptimize(solver.matrixLU().data());
  }
}

static void BM_HouseholderQR(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  Mat A = Mat::Random(rows, cols);
  HouseholderQR<Mat> solver(rows, cols);
  for (auto _ : state) {
    compute(solver, A);
    benchmark::DoNotOptimize(solver.matrixQR().data());
  }
}

static void BM_ColPivHouseholderQR(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  Mat A = Mat::Random(rows, cols);
  ColPivHouseholderQR<Mat> solver(rows, cols);
  for (auto _ : state) {
    compute(solver, A);
    benchmark::DoNotOptimize(solver.matrixQR().data());
  }
}

static void BM_COD(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  Mat A = Mat::Random(rows, cols);
  CompleteOrthogonalDecomposition<Mat> solver(rows, cols);
  for (auto _ : state) {
    compute(solver, A);
    benchmark::DoNotOptimize(solver.matrixQTZ().data());
  }
}

static void BM_FullPivHouseholderQR(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  Mat A = Mat::Random(rows, cols);
  FullPivHouseholderQR<Mat> solver(rows, cols);
  for (auto _ : state) {
    compute(solver, A);
    benchmark::DoNotOptimize(solver.matrixQR().data());
  }
}

static void BM_JacobiSVD(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  Mat A = Mat::Random(rows, cols);
  JacobiSVD<Mat, ComputeThinU | ComputeThinV> solver(rows, cols);
  for (auto _ : state) {
    solver.compute(A);
    benchmark::DoNotOptimize(solver.singularValues().data());
  }
}

static void BM_BDCSVD(benchmark::State& state) {
  int rows = state.range(0);
  int cols = state.range(1);
  Mat A = Mat::Random(rows, cols);
  BDCSVD<Mat, ComputeThinU | ComputeThinV> solver(rows, cols);
  for (auto _ : state) {
    solver.compute(A);
    benchmark::DoNotOptimize(solver.singularValues().data());
  }
}

static void DenseSolverSizes(::benchmark::Benchmark* b) {
  // Square sizes
  for (int s : {8, 100, 1000}) {
    b->Args({s, s});
  }
  // Tall-skinny sizes
  b->Args({10000, 8});
  b->Args({10000, 100});
}

BENCHMARK(BM_LLT)->Apply(DenseSolverSizes);
BENCHMARK(BM_LDLT)->Apply(DenseSolverSizes);
BENCHMARK(BM_PartialPivLU)->Apply(DenseSolverSizes);
BENCHMARK(BM_FullPivLU)->Apply(DenseSolverSizes);
BENCHMARK(BM_HouseholderQR)->Apply(DenseSolverSizes);
BENCHMARK(BM_ColPivHouseholderQR)->Apply(DenseSolverSizes);
BENCHMARK(BM_COD)->Apply(DenseSolverSizes);
BENCHMARK(BM_FullPivHouseholderQR)->Apply(DenseSolverSizes);
BENCHMARK(BM_JacobiSVD)->Apply([](::benchmark::Benchmark* b) {
  // JacobiSVD is very slow for large matrices
  for (int s : {8, 100}) b->Args({s, s});
  b->Args({10000, 8});
});
BENCHMARK(BM_BDCSVD)->Apply(DenseSolverSizes);
