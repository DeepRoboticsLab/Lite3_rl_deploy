#include <benchmark/benchmark.h>
#include <Eigen/Sparse>
#include <set>

using namespace Eigen;

typedef double Scalar;
typedef SparseMatrix<Scalar> SpMat;

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

static void BM_SparseMM(benchmark::State& state) {
  int n = state.range(0);
  int nnzPerCol = state.range(1);
  SpMat sm1(n, n), sm2(n, n), sm3(n, n);
  fillMatrix(nnzPerCol, n, n, sm1);
  fillMatrix(nnzPerCol, n, n, sm2);
  for (auto _ : state) {
    sm3 = sm1 * sm2;
    benchmark::DoNotOptimize(sm3.valuePtr());
  }
  state.counters["nnz_A"] = sm1.nonZeros();
  state.counters["nnz_B"] = sm2.nonZeros();
}

static void SpMMSizes(::benchmark::Benchmark* b) {
  for (int n : {1000, 10000}) {
    for (int nnz : {4, 6, 10}) {
      b->Args({n, nnz});
    }
  }
}

BENCHMARK(BM_SparseMM)->Apply(SpMMSizes);
