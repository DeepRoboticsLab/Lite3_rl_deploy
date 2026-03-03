#include <benchmark/benchmark.h>

#include <cstdint>

bool eigen_use_specific_block_size;
int eigen_block_size_k, eigen_block_size_m, eigen_block_size_n;
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZES eigen_use_specific_block_size
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZE_K eigen_block_size_k
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZE_M eigen_block_size_m
#define EIGEN_TEST_SPECIFIC_BLOCKING_SIZE_N eigen_block_size_n
#include <Eigen/Core>

using namespace Eigen;

typedef MatrixXf MatrixType;
typedef MatrixType::Scalar Scalar;

static void BM_GemmDefaultBlocking(benchmark::State& state) {
  int k = state.range(0);
  int m = state.range(1);
  int n = state.range(2);
  eigen_use_specific_block_size = false;
  MatrixType lhs = MatrixType::Random(m, k);
  MatrixType rhs = MatrixType::Random(k, n);
  MatrixType dst = MatrixType::Zero(m, n);
  for (auto _ : state) {
    dst.noalias() = lhs * rhs;
    benchmark::DoNotOptimize(dst.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * k * m * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

static void BM_GemmCustomBlocking(benchmark::State& state) {
  int k = state.range(0);
  int m = state.range(1);
  int n = state.range(2);
  int bk = state.range(3);
  int bm = state.range(4);
  int bn = state.range(5);
  eigen_use_specific_block_size = true;
  eigen_block_size_k = bk;
  eigen_block_size_m = bm;
  eigen_block_size_n = bn;
  MatrixType lhs = MatrixType::Random(m, k);
  MatrixType rhs = MatrixType::Random(k, n);
  MatrixType dst = MatrixType::Zero(m, n);
  for (auto _ : state) {
    dst.noalias() = lhs * rhs;
    benchmark::DoNotOptimize(dst.data());
  }
  state.counters["GFLOPS"] =
      benchmark::Counter(2.0 * k * m * n, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

static void DefaultBlockingSizes(::benchmark::Benchmark* b) {
  for (int s : {64, 128, 256, 512, 1024, 2048}) {
    b->Args({s, s, s});
  }
}

static void CustomBlockingSizes(::benchmark::Benchmark* b) {
  // Test a few product sizes with varying block sizes
  for (int s : {256, 512, 1024}) {
    for (int bk : {16, 32, 64, 128, 256}) {
      if (bk > s) continue;
      for (int bm : {16, 32, 64, 128, 256}) {
        if (bm > s) continue;
        b->Args({s, s, s, bk, bm, s});
      }
    }
  }
}

BENCHMARK(BM_GemmDefaultBlocking)->Apply(DefaultBlockingSizes);
BENCHMARK(BM_GemmCustomBlocking)->Apply(CustomBlockingSizes);
