#include <benchmark/benchmark.h>
#include <Eigen/Geometry>

using namespace Eigen;

// Helper to get dimension from various transform types
template <typename T>
struct TransformDim {
  static constexpr int value = T::Dim;
};

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct TransformDim<Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
  static constexpr int value = Rows;
};

template <typename Transformation, int N>
static void BM_TransformData(benchmark::State& state) {
  typedef typename Transformation::Scalar Scalar;
  constexpr int Dim = TransformDim<Transformation>::value;
  Transformation t;
  if constexpr (std::is_same_v<Transformation, Matrix<Scalar, Dim, Dim>>) {
    t.setRandom();
  } else {
    Matrix<Scalar, Dim, Dim + 1> mat;
    mat.setRandom();
    t = Transformation(mat);
  }
  Matrix<Scalar, Dim, N> data;
  data.setRandom();
  for (auto _ : state) {
    data = t * data;
    benchmark::DoNotOptimize(data.data());
  }
}

// For quaternion: apply per-column
template <typename Scalar, int N>
static void BM_QuatTransform(benchmark::State& state) {
  Quaternion<Scalar> q;
  q.setIdentity();
  Matrix<Scalar, 3, N> data;
  data.setRandom();
  for (auto _ : state) {
    for (int i = 0; i < N; ++i) data.col(i) = q * data.col(i);
    benchmark::DoNotOptimize(data.data());
  }
}

// Use typedefs to avoid commas in macro arguments
typedef Transform<float, 3, Isometry> Isometry3f_t;
typedef Transform<float, 3, Affine> Affine3f_t;
typedef Transform<float, 3, AffineCompact> AffineCompact3f_t;
typedef Matrix<float, 3, 3> Matrix3f_t;

BENCHMARK(BM_TransformData<Isometry3f_t, 1>);
BENCHMARK(BM_TransformData<Isometry3f_t, 4>);
BENCHMARK(BM_TransformData<Isometry3f_t, 8>);
BENCHMARK(BM_TransformData<Affine3f_t, 1>);
BENCHMARK(BM_TransformData<Affine3f_t, 4>);
BENCHMARK(BM_TransformData<Affine3f_t, 8>);
BENCHMARK(BM_TransformData<AffineCompact3f_t, 1>);
BENCHMARK(BM_TransformData<AffineCompact3f_t, 4>);
BENCHMARK(BM_TransformData<AffineCompact3f_t, 8>);
BENCHMARK(BM_TransformData<Matrix3f_t, 1>);
BENCHMARK(BM_TransformData<Matrix3f_t, 4>);
BENCHMARK(BM_TransformData<Matrix3f_t, 8>);

BENCHMARK(BM_QuatTransform<float, 1>);
BENCHMARK(BM_QuatTransform<float, 4>);
BENCHMARK(BM_QuatTransform<float, 8>);
