#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

using namespace Eigen;

#ifndef SCALAR
#define SCALAR float
#endif

typedef SCALAR Scalar;

static void BM_SelfAdjointEigenSolver(benchmark::State& state) {
  int n = state.range(0);
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  MatrixType a = MatrixType::Random(n, n);
  MatrixType covMat = a * a.adjoint();
  int r = internal::random<int>(0, n - 1);
  int c = internal::random<int>(0, n - 1);
  Scalar acc = 0;
  SelfAdjointEigenSolver<MatrixType> ei(covMat);
  for (auto _ : state) {
    ei.compute(covMat);
    acc += ei.eigenvectors().coeff(r, c);
    benchmark::DoNotOptimize(acc);
  }
}
BENCHMARK(BM_SelfAdjointEigenSolver)->RangeMultiplier(2)->Range(4, 512);

static void BM_EigenSolver(benchmark::State& state) {
  int n = state.range(0);
  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixType;
  MatrixType a = MatrixType::Random(n, n);
  MatrixType covMat = a * a.adjoint();
  int r = internal::random<int>(0, n - 1);
  int c = internal::random<int>(0, n - 1);
  Scalar acc = 0;
  EigenSolver<MatrixType> ei(covMat);
  for (auto _ : state) {
    ei.compute(covMat);
    acc += std::norm(ei.eigenvectors().coeff(r, c));
    benchmark::DoNotOptimize(acc);
  }
}
BENCHMARK(BM_EigenSolver)->RangeMultiplier(2)->Range(4, 512);
