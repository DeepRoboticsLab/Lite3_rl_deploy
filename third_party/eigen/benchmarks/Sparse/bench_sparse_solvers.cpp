// Benchmarks for sparse decomposition solvers.
// Tests SimplicialLLT, SimplicialLDLT, SparseQR, SparseLU, CG, BiCGSTAB.

#include <benchmark/benchmark.h>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/OrderingMethods>

using namespace Eigen;

typedef double Scalar;
typedef SparseMatrix<Scalar> SpMat;
typedef Matrix<Scalar, Dynamic, 1> Vec;

// Generate a SPD banded matrix (Laplacian-like).
static SpMat generateSPD(int n, int bandwidth) {
  SpMat A(n, n);
  std::vector<Triplet<Scalar>> trips;
  trips.reserve(n * (2 * bandwidth + 1));
  for (int i = 0; i < n; ++i) {
    Scalar diag = 0;
    for (int j = std::max(0, i - bandwidth); j < std::min(n, i + bandwidth + 1); ++j) {
      if (i != j) {
        Scalar val = -1.0 / (1 + std::abs(i - j));
        trips.emplace_back(i, j, val);
        diag -= val;
      }
    }
    trips.emplace_back(i, i, diag + 1.0);
  }
  A.setFromTriplets(trips.begin(), trips.end());
  return A;
}

// Generate a general (non-symmetric) sparse matrix with diagonal dominance.
static SpMat generateGeneral(int n, int bandwidth) {
  SpMat A(n, n);
  std::vector<Triplet<Scalar>> trips;
  trips.reserve(n * (2 * bandwidth + 1));
  for (int i = 0; i < n; ++i) {
    Scalar diag = 0;
    for (int j = std::max(0, i - bandwidth); j < std::min(n, i + bandwidth + 1); ++j) {
      if (i != j) {
        Scalar val = -0.5 / (1 + std::abs(i - j));
        if (j > i) val *= 1.5;
        trips.emplace_back(i, j, val);
        diag += std::abs(val);
      }
    }
    trips.emplace_back(i, i, diag + 1.0);
  }
  A.setFromTriplets(trips.begin(), trips.end());
  return A;
}

// --- SimplicialLLT ---
static void BM_SimplicialLLT(benchmark::State& state) {
  int n = state.range(0);
  int bw = state.range(1);
  SpMat A = generateSPD(n, bw);
  Vec b = Vec::Random(n);

  for (auto _ : state) {
    SimplicialLLT<SpMat> solver(A);
    Vec x = solver.solve(b);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
}

// --- SimplicialLDLT ---
static void BM_SimplicialLDLT(benchmark::State& state) {
  int n = state.range(0);
  int bw = state.range(1);
  SpMat A = generateSPD(n, bw);
  Vec b = Vec::Random(n);

  for (auto _ : state) {
    SimplicialLDLT<SpMat> solver(A);
    Vec x = solver.solve(b);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
}

// --- SparseLU ---
static void BM_SparseLU(benchmark::State& state) {
  int n = state.range(0);
  int bw = state.range(1);
  SpMat A = generateGeneral(n, bw);
  Vec b = Vec::Random(n);

  for (auto _ : state) {
    SparseLU<SpMat, COLAMDOrdering<int>> solver;
    solver.compute(A);
    Vec x = solver.solve(b);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
}

// --- SparseQR ---
static void BM_SparseQR(benchmark::State& state) {
  int n = state.range(0);
  int bw = state.range(1);
  SpMat A = generateGeneral(n, bw);
  Vec b = Vec::Random(n);

  for (auto _ : state) {
    SparseQR<SpMat, COLAMDOrdering<int>> solver;
    solver.compute(A);
    Vec x = solver.solve(b);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
}

// --- ConjugateGradient (SPD) ---
static void BM_CG(benchmark::State& state) {
  int n = state.range(0);
  int bw = state.range(1);
  SpMat A = generateSPD(n, bw);
  Vec b = Vec::Random(n);

  ConjugateGradient<SpMat> solver;
  solver.setMaxIterations(1000);
  solver.setTolerance(1e-10);
  solver.compute(A);

  for (auto _ : state) {
    Vec x = solver.solve(b);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
  state.counters["iterations"] = solver.iterations();
}

// --- BiCGSTAB (general) ---
static void BM_BiCGSTAB(benchmark::State& state) {
  int n = state.range(0);
  int bw = state.range(1);
  SpMat A = generateGeneral(n, bw);
  Vec b = Vec::Random(n);

  BiCGSTAB<SpMat> solver;
  solver.setMaxIterations(1000);
  solver.setTolerance(1e-10);
  solver.compute(A);

  for (auto _ : state) {
    Vec x = solver.solve(b);
    benchmark::DoNotOptimize(x.data());
    benchmark::ClobberMemory();
  }
  state.counters["iterations"] = solver.iterations();
}

static void DirectSolverSizes(::benchmark::Benchmark* b) {
  for (int n : {1000, 5000, 10000, 50000}) {
    for (int bw : {5, 20}) {
      b->Args({n, bw});
    }
  }
}

static void IterativeSolverSizes(::benchmark::Benchmark* b) {
  for (int n : {1000, 10000, 50000}) {
    for (int bw : {5, 20}) {
      b->Args({n, bw});
    }
  }
}

BENCHMARK(BM_SimplicialLLT)->Apply(DirectSolverSizes);
BENCHMARK(BM_SimplicialLDLT)->Apply(DirectSolverSizes);
BENCHMARK(BM_SparseLU)->Apply(DirectSolverSizes);
BENCHMARK(BM_SparseQR)->Apply(DirectSolverSizes);
BENCHMARK(BM_CG)->Apply(IterativeSolverSizes);
BENCHMARK(BM_BiCGSTAB)->Apply(IterativeSolverSizes);
