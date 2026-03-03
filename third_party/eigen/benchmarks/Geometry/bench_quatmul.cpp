#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

template <typename Quat>
EIGEN_DONT_INLINE void quatmul_default(const Quat& a, const Quat& b, Quat& c) {
  c = a * b;
}

template <typename Quat>
EIGEN_DONT_INLINE void quatmul_novec(const Quat& a, const Quat& b, Quat& c) {
  c = internal::quat_product<0, Quat, Quat, typename Quat::Scalar>::run(a, b);
}

template <typename Quat>
static void BM_QuatMul_Default(benchmark::State& state) {
  Quat a(4, 1, 2, 3), b(2, 3, 4, 5), c;
  for (auto _ : state) {
    quatmul_default(a, b, c);
    benchmark::DoNotOptimize(c.coeffs().data());
  }
}

template <typename Quat>
static void BM_QuatMul_NoVec(benchmark::State& state) {
  Quat a(4, 1, 2, 3), b(2, 3, 4, 5), c;
  for (auto _ : state) {
    quatmul_novec(a, b, c);
    benchmark::DoNotOptimize(c.coeffs().data());
  }
}

BENCHMARK(BM_QuatMul_Default<Quaternionf>);
BENCHMARK(BM_QuatMul_NoVec<Quaternionf>);
BENCHMARK(BM_QuatMul_Default<Quaterniond>);
BENCHMARK(BM_QuatMul_NoVec<Quaterniond>);
