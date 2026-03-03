#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace Eigen;

static void BM_VectorExp(benchmark::State& state) {
  int n = state.range(0);
  VectorXd v = VectorXd::LinSpaced(n, 0.1, 10.0);
  VectorXd result(n);
  for (auto _ : state) {
    result = v.array().exp();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(double));
}

static void BM_VectorSin(benchmark::State& state) {
  int n = state.range(0);
  VectorXd v = VectorXd::LinSpaced(n, 0.1, 10.0);
  VectorXd result(n);
  for (auto _ : state) {
    result = v.array().sin();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(double));
}

static void BM_VectorCos(benchmark::State& state) {
  int n = state.range(0);
  VectorXd v = VectorXd::LinSpaced(n, 0.1, 10.0);
  VectorXd result(n);
  for (auto _ : state) {
    result = v.array().cos();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(double));
}

static void BM_VectorSqrt(benchmark::State& state) {
  int n = state.range(0);
  VectorXd v = VectorXd::LinSpaced(n, 0.1, 10.0);
  VectorXd result(n);
  for (auto _ : state) {
    result = v.array().sqrt();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(double));
}

static void BM_VectorLog(benchmark::State& state) {
  int n = state.range(0);
  VectorXd v = VectorXd::LinSpaced(n, 0.1, 10.0);
  VectorXd result(n);
  for (auto _ : state) {
    result = v.array().log();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(double));
}

static void BM_VectorTanh(benchmark::State& state) {
  int n = state.range(0);
  VectorXd v = VectorXd::LinSpaced(n, 0.1, 10.0);
  VectorXd result(n);
  for (auto _ : state) {
    result = v.array().tanh();
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(state.iterations() * n * sizeof(double));
}

static void VectorSizes(::benchmark::Benchmark* b) {
  for (int n : {10000, 100000, 1000000, 5000000}) {
    b->Arg(n);
  }
}

BENCHMARK(BM_VectorExp)->Apply(VectorSizes);
BENCHMARK(BM_VectorSin)->Apply(VectorSizes);
BENCHMARK(BM_VectorCos)->Apply(VectorSizes);
BENCHMARK(BM_VectorSqrt)->Apply(VectorSizes);
BENCHMARK(BM_VectorLog)->Apply(VectorSizes);
BENCHMARK(BM_VectorTanh)->Apply(VectorSizes);

static void BM_DGEMM(benchmark::State& state) {
  int n = state.range(0);
  MatrixXd A = MatrixXd::Random(n, n);
  MatrixXd B = MatrixXd::Random(n, n);
  MatrixXd C(n, n);
  for (auto _ : state) {
    C.noalias() = A * B;
    benchmark::DoNotOptimize(C.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * n * n * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}
BENCHMARK(BM_DGEMM)->Arg(256)->Arg(512)->Arg(1024)->Arg(2048);

static void BM_EigenDecomposition(benchmark::State& state) {
  int n = state.range(0);
  MatrixXd M = MatrixXd::Random(n, n);
  M = (M + M.transpose()) * 0.5;
  SelfAdjointEigenSolver<MatrixXd> solver;
  for (auto _ : state) {
    solver.compute(M);
    benchmark::DoNotOptimize(solver.eigenvalues().data());
  }
}
BENCHMARK(BM_EigenDecomposition)->Arg(256)->Arg(512)->Arg(1024);

static void BM_FSI_Risk(benchmark::State& state) {
  int numPeriods = state.range(0);
  int numAssets = state.range(1);
  MatrixXd returns = MatrixXd::Random(numPeriods, numAssets);
  for (auto _ : state) {
    MatrixXd cov = (returns.transpose() * returns) / (numPeriods - 1);
    SelfAdjointEigenSolver<MatrixXd> solver(cov);
    benchmark::DoNotOptimize(solver.eigenvalues().data());
  }
}
BENCHMARK(BM_FSI_Risk)->Args({10000, 500});
