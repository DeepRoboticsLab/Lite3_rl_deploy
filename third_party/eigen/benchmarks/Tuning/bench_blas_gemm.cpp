// Benchmark: Eigen GEMM vs CBLAS GEMM
// Requires CBLAS: compile with -DHAVE_BLAS and link -lcblas
//
// Based on the old bench/benchBlasGemm.cpp (removed)

#include <benchmark/benchmark.h>
#include <Eigen/Core>

using namespace Eigen;

#ifndef SCALAR
#define SCALAR float
#endif

typedef SCALAR Scalar;
typedef Matrix<Scalar, Dynamic, Dynamic> MyMatrix;

static void BM_EigenGemm(benchmark::State& state) {
  int M = state.range(0);
  int N = state.range(1);
  int K = state.range(2);
  MyMatrix a = MyMatrix::Random(M, K);
  MyMatrix b = MyMatrix::Random(K, N);
  MyMatrix c = MyMatrix::Random(M, N);
  for (auto _ : state) {
    c.noalias() += a * b;
    benchmark::DoNotOptimize(c.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * M * N * K, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

#ifdef HAVE_BLAS
extern "C" {
#include <cblas.h>
}

#ifdef _FLOAT
#define CBLAS_GEMM cblas_sgemm
#else
#define CBLAS_GEMM cblas_dgemm
#endif

static void BM_CblasGemm(benchmark::State& state) {
  int M = state.range(0);
  int N = state.range(1);
  int K = state.range(2);
  MyMatrix a = MyMatrix::Random(M, K);
  MyMatrix b = MyMatrix::Random(K, N);
  MyMatrix c = MyMatrix::Random(M, N);
  Scalar alpha = 1, beta = 1;
  for (auto _ : state) {
    CBLAS_GEMM(CblasColMajor, CblasNoTrans, CblasNoTrans, M, N, K, alpha, a.data(), M, b.data(), K, beta, c.data(), M);
    benchmark::DoNotOptimize(c.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * M * N * K, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}
#endif

static void GemmSizes(::benchmark::Benchmark* b) {
  for (int s : {32, 64, 128, 256, 512, 1024, 2048}) {
    b->Args({s, s, s});
  }
  // Rectangular
  b->Args({1000, 100, 1000});
  b->Args({100, 1000, 100});
}

BENCHMARK(BM_EigenGemm)->Apply(GemmSizes);
#ifdef HAVE_BLAS
BENCHMARK(BM_CblasGemm)->Apply(GemmSizes);
#endif
