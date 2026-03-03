#include <benchmark/benchmark.h>
#include <Eigen/Sparse>

using namespace Eigen;

typedef double Scalar;
typedef SparseMatrix<Scalar> SpMat;
typedef Matrix<Scalar, Dynamic, 1> DenseVec;

static void fillMatrix(int nnzPerCol, int rows, int cols, SpMat& dst) {
  dst.resize(rows, cols);
  dst.reserve(VectorXi::Constant(cols, nnzPerCol));
  for (int j = 0; j < cols; ++j) {
    std::set<int> used;
    for (int i = 0; i < nnzPerCol; ++i) {
      int row;
      do {
        row = internal::random<int>(0, rows - 1);
      } while (used.count(row));
      used.insert(row);
      dst.insert(row, j) = internal::random<Scalar>();
    }
  }
  dst.makeCompressed();
}

static void BM_SpMV(benchmark::State& state) {
  int n = state.range(0);
  int nnzPerCol = state.range(1);
  SpMat sm(n, n);
  fillMatrix(nnzPerCol, n, n, sm);
  DenseVec v = DenseVec::Random(n);
  DenseVec res(n);
  for (auto _ : state) {
    res.noalias() = sm * v;
    benchmark::DoNotOptimize(res.data());
  }
  state.counters["nnz"] = sm.nonZeros();
}

static void BM_SpMV_Transpose(benchmark::State& state) {
  int n = state.range(0);
  int nnzPerCol = state.range(1);
  SpMat sm(n, n);
  fillMatrix(nnzPerCol, n, n, sm);
  DenseVec v = DenseVec::Random(n);
  DenseVec res(n);
  for (auto _ : state) {
    res.noalias() = sm.transpose() * v;
    benchmark::DoNotOptimize(res.data());
  }
  state.counters["nnz"] = sm.nonZeros();
}

static void SpMVSizes(::benchmark::Benchmark* b) {
  for (int n : {1000, 10000, 100000}) {
    for (int nnz : {7, 20, 50}) {
      b->Args({n, nnz});
    }
  }
}

BENCHMARK(BM_SpMV)->Apply(SpMVSizes);
BENCHMARK(BM_SpMV_Transpose)->Apply(SpMVSizes);
