// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen (rmlarsen@gmail.com)
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Benchmarks for reciprocal condition number estimation.
//
// Times the rcond() call on pre-computed factorizations.
// The rcond estimator is O(n^2), so this isolates its cost from the O(n^3) factorization.

#include <benchmark/benchmark.h>
#include <Eigen/Dense>

using namespace Eigen;

// --- PartialPivLU ---

template <typename Scalar>
static void BM_PartialPivLU_Rcond(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  A.diagonal().array() += Scalar(2 * n);
  PartialPivLU<Mat> lu(A);
  for (auto _ : state) {
    auto rc = lu.rcond();
    benchmark::DoNotOptimize(rc);
  }
  state.SetItemsProcessed(state.iterations());
}

// --- FullPivLU ---

template <typename Scalar>
static void BM_FullPivLU_Rcond(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  A.diagonal().array() += Scalar(2 * n);
  FullPivLU<Mat> lu(A);
  for (auto _ : state) {
    auto rc = lu.rcond();
    benchmark::DoNotOptimize(rc);
  }
  state.SetItemsProcessed(state.iterations());
}

// --- LLT ---

template <typename Scalar>
static void BM_LLT_Rcond(benchmark::State& state) {
  const Index n = state.range(0);
  using Mat = Matrix<Scalar, Dynamic, Dynamic>;
  Mat A = Mat::Random(n, n);
  Mat SPD = A.adjoint() * A + Mat::Identity(n, n);
  LLT<Mat> llt(SPD);
  for (auto _ : state) {
    auto rc = llt.rcond();
    benchmark::DoNotOptimize(rc);
  }
  state.SetItemsProcessed(state.iterations());
}

// --- Size configurations ---

static void SquareSizes(::benchmark::Benchmark* b) {
  for (int n : {8, 32, 64, 128, 256, 512, 1024}) b->Arg(n);
}

BENCHMARK(BM_PartialPivLU_Rcond<float>)->Apply(SquareSizes)->Name("PartialPivLU_Rcond_float");
BENCHMARK(BM_PartialPivLU_Rcond<double>)->Apply(SquareSizes)->Name("PartialPivLU_Rcond_double");
BENCHMARK(BM_FullPivLU_Rcond<float>)->Apply(SquareSizes)->Name("FullPivLU_Rcond_float");
BENCHMARK(BM_FullPivLU_Rcond<double>)->Apply(SquareSizes)->Name("FullPivLU_Rcond_double");
BENCHMARK(BM_LLT_Rcond<float>)->Apply(SquareSizes)->Name("LLT_Rcond_float");
BENCHMARK(BM_LLT_Rcond<double>)->Apply(SquareSizes)->Name("LLT_Rcond_double");
