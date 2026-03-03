#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <unsupported/Eigen/FFT>
#include <complex>
#include <vector>

using namespace Eigen;

template <typename T, bool Forward, bool Unscaled = false, bool HalfSpec = false>
static void BM_FFT(benchmark::State& state) {
  typedef typename NumTraits<T>::Real ScalarType;
  typedef std::complex<ScalarType> Complex;
  int nfft = state.range(0);
  std::vector<T> inbuf(nfft);
  std::vector<Complex> outbuf(nfft);
  FFT<ScalarType> fft;
  if (Unscaled) fft.SetFlag(fft.Unscaled);
  if (HalfSpec) fft.SetFlag(fft.HalfSpectrum);
  std::fill(inbuf.begin(), inbuf.end(), T(0));
  fft.fwd(outbuf, inbuf);

  for (auto _ : state) {
    if (Forward)
      fft.fwd(outbuf, inbuf);
    else
      fft.inv(inbuf, outbuf);
    benchmark::DoNotOptimize(outbuf.data());
    benchmark::DoNotOptimize(inbuf.data());
  }
  double mflops_per_iter = 5.0 * nfft * std::log2(static_cast<double>(nfft));
  if (!NumTraits<T>::IsComplex) mflops_per_iter /= 2;
  state.counters["MFLOPS"] =
      benchmark::Counter(mflops_per_iter, benchmark::Counter::kIsIterationInvariantRate, benchmark::Counter::kIs1000);
}

static void FFTSizes(::benchmark::Benchmark* b) {
  for (int n : {64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536}) {
    b->Arg(n);
  }
  // Non-power-of-2 sizes.
  b->Arg(1000);
  b->Arg(5000);
}

BENCHMARK(BM_FFT<std::complex<float>, true>)->Apply(FFTSizes);
BENCHMARK(BM_FFT<std::complex<float>, false>)->Apply(FFTSizes);
BENCHMARK(BM_FFT<float, true>)->Apply(FFTSizes);
BENCHMARK(BM_FFT<float, false>)->Apply(FFTSizes);
BENCHMARK(BM_FFT<std::complex<double>, true>)->Apply(FFTSizes);
BENCHMARK(BM_FFT<std::complex<double>, false>)->Apply(FFTSizes);
BENCHMARK(BM_FFT<double, true>)->Apply(FFTSizes);
BENCHMARK(BM_FFT<double, false>)->Apply(FFTSizes);
