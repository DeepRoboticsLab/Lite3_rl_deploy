// Standalone JacobiSVD benchmark for block-size and threshold sweeps.
// Compile-time parameters via -D flags:
//   BLOCK_SIZE (default 24), BLOCKING_THRESHOLD (default 192)
//
// Build example:
//   g++ -O3 -DNDEBUG -march=native -DBLOCK_SIZE=16 -DBLOCKING_THRESHOLD=128
//       -I../.. bench_jacobi_sweep.cpp -lbenchmark -lbenchmark_main -lpthread

#include <benchmark/benchmark.h>

// Override block size before including Eigen headers.
#ifdef BLOCK_SIZE
#define EIGEN_JACOBI_SVD_BLOCK_SIZE BLOCK_SIZE
#endif
#ifdef BLOCKING_THRESHOLD
#define EIGEN_JACOBI_SVD_BLOCKING_THRESHOLD BLOCKING_THRESHOLD
#endif

#include <Eigen/Dense>

using namespace Eigen;

template <typename Scalar>
using Mat = Matrix<Scalar, Dynamic, Dynamic>;

template <typename SVD>
EIGEN_DONT_INLINE void do_compute(SVD& svd, const typename SVD::MatrixType& A) {
  svd.compute(A);
}

template <typename Scalar, int Options>
static void BM_JacobiSVD(benchmark::State& state) {
  const Index n = state.range(0);
  Mat<Scalar> A = Mat<Scalar>::Random(n, n);
  JacobiSVD<Mat<Scalar>, Options> svd(n, n);
  for (auto _ : state) {
    do_compute(svd, A);
    benchmark::DoNotOptimize(svd.singularValues().data());
  }
  state.SetItemsProcessed(state.iterations());
}

static void Sizes(::benchmark::Benchmark* b) {
  for (int s : {32, 64, 128, 192, 256, 384, 512}) b->Args({s});
}

// float ValuesOnly
BENCHMARK(BM_JacobiSVD<float, 0>)->Apply(Sizes)->Name("Jacobi_float_VO");
// float ThinUV
BENCHMARK(BM_JacobiSVD<float, ComputeThinU | ComputeThinV>)->Apply(Sizes)->Name("Jacobi_float_UV");
// double ValuesOnly
BENCHMARK(BM_JacobiSVD<double, 0>)->Apply(Sizes)->Name("Jacobi_double_VO");
// double ThinUV
BENCHMARK(BM_JacobiSVD<double, ComputeThinU | ComputeThinV>)->Apply(Sizes)->Name("Jacobi_double_UV");
// complex<float> ValuesOnly
BENCHMARK(BM_JacobiSVD<std::complex<float>, 0>)->Apply(Sizes)->Name("Jacobi_cfloat_VO");
// complex<float> ThinUV
BENCHMARK((BM_JacobiSVD<std::complex<float>, ComputeThinU | ComputeThinV>))->Apply(Sizes)->Name("Jacobi_cfloat_UV");
// complex<double> ValuesOnly
BENCHMARK(BM_JacobiSVD<std::complex<double>, 0>)->Apply(Sizes)->Name("Jacobi_cdouble_VO");
// complex<double> ThinUV
BENCHMARK((BM_JacobiSVD<std::complex<double>, ComputeThinU | ComputeThinV>))->Apply(Sizes)->Name("Jacobi_cdouble_UV");
