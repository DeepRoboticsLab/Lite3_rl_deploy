#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

using namespace Eigen;

template <typename Matrix, typename Roots>
inline void computeRoots(const Matrix& m, Roots& roots) {
  typedef typename Matrix::Scalar Scalar;
  const Scalar s_inv3 = 1.0 / 3.0;
  const Scalar s_sqrt3 = std::sqrt(Scalar(3.0));
  Scalar c0 = m(0, 0) * m(1, 1) * m(2, 2) + Scalar(2) * m(0, 1) * m(0, 2) * m(1, 2) - m(0, 0) * m(1, 2) * m(1, 2) -
              m(1, 1) * m(0, 2) * m(0, 2) - m(2, 2) * m(0, 1) * m(0, 1);
  Scalar c1 = m(0, 0) * m(1, 1) - m(0, 1) * m(0, 1) + m(0, 0) * m(2, 2) - m(0, 2) * m(0, 2) + m(1, 1) * m(2, 2) -
              m(1, 2) * m(1, 2);
  Scalar c2 = m(0, 0) + m(1, 1) + m(2, 2);
  Scalar c2_over_3 = c2 * s_inv3;
  Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
  if (a_over_3 > Scalar(0)) a_over_3 = Scalar(0);
  Scalar half_b = Scalar(0.5) * (c0 + c2_over_3 * (Scalar(2) * c2_over_3 * c2_over_3 - c1));
  Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
  if (q > Scalar(0)) q = Scalar(0);
  Scalar rho = std::sqrt(-a_over_3);
  Scalar theta = std::atan2(std::sqrt(-q), half_b) * s_inv3;
  Scalar cos_theta = std::cos(theta);
  Scalar sin_theta = std::sin(theta);
  roots(2) = c2_over_3 + Scalar(2) * rho * cos_theta;
  roots(0) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
  roots(1) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);
}

template <typename Matrix, typename Vector>
void eigen33(const Matrix& mat, Matrix& evecs, Vector& evals) {
  typedef typename Matrix::Scalar Scalar;
  Scalar shift = mat.trace() / 3;
  Matrix scaledMat = mat;
  scaledMat.diagonal().array() -= shift;
  Scalar scale = scaledMat.cwiseAbs().maxCoeff();
  scale = std::max(scale, Scalar(1));
  scaledMat /= scale;
  computeRoots(scaledMat, evals);
  if ((evals(2) - evals(0)) <= Eigen::NumTraits<Scalar>::epsilon()) {
    evecs.setIdentity();
  } else {
    Matrix tmp;
    tmp = scaledMat;
    tmp.diagonal().array() -= evals(2);
    evecs.col(2) = tmp.row(0).cross(tmp.row(1)).normalized();
    tmp = scaledMat;
    tmp.diagonal().array() -= evals(1);
    evecs.col(1) = tmp.row(0).cross(tmp.row(1));
    Scalar n1 = evecs.col(1).norm();
    if (n1 <= Eigen::NumTraits<Scalar>::epsilon())
      evecs.col(1) = evecs.col(2).unitOrthogonal();
    else
      evecs.col(1) /= n1;
    evecs.col(1) = evecs.col(2).cross(evecs.col(1).cross(evecs.col(2))).normalized();
    evecs.col(0) = evecs.col(2).cross(evecs.col(1));
  }
  evals *= scale;
  evals.array() += shift;
}

static void BM_Eig33_Iterative(benchmark::State& state) {
  Matrix3d A = Matrix3d::Random();
  A = A.adjoint() * A;
  SelfAdjointEigenSolver<Matrix3d> eig(A);
  for (auto _ : state) {
    eig.compute(A);
    benchmark::DoNotOptimize(eig.eigenvalues().data());
  }
}
BENCHMARK(BM_Eig33_Iterative);

static void BM_Eig33_Direct(benchmark::State& state) {
  Matrix3d A = Matrix3d::Random();
  A = A.adjoint() * A;
  SelfAdjointEigenSolver<Matrix3d> eig(A);
  for (auto _ : state) {
    eig.computeDirect(A);
    benchmark::DoNotOptimize(eig.eigenvalues().data());
  }
}
BENCHMARK(BM_Eig33_Direct);

static void BM_Eig33_Custom(benchmark::State& state) {
  Matrix3d A = Matrix3d::Random();
  A = A.adjoint() * A;
  Matrix3d evecs;
  Vector3d evals;
  for (auto _ : state) {
    eigen33(A, evecs, evals);
    benchmark::DoNotOptimize(evals.data());
  }
}
BENCHMARK(BM_Eig33_Custom);
