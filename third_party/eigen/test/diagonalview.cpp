// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

// discard stack allocation as that too bypasses malloc
#define EIGEN_STACK_ALLOCATION_LIMIT 0
// heap allocation will raise an assert if enabled at runtime
#define EIGEN_RUNTIME_NO_MALLOC

#include "main.h"
using namespace std;
template <typename MatrixType>
void diagonalview(const MatrixType& m) {
  enum { Rows = MatrixType::RowsAtCompileTime, Cols = MatrixType::ColsAtCompileTime };
  Index rows = m.rows();
  Index cols = m.cols();
  // create random matrix
  MatrixType m1 = MatrixType::Random(rows, cols);

  // check equivalence to diagonal(i).asDiagonal() for dynamic indexes
  VERIFY_IS_APPROX(m1.diagonal(0).asDiagonal().toDenseMatrix(), m1.diagonalView(0).toDenseMatrix());
  // subdiagonal
  VERIFY_IS_APPROX(m1.diagonal(-1).asDiagonal().toDenseMatrix(), m1.diagonalView(-1).toDenseMatrix());
  // superdiagonal
  VERIFY_IS_APPROX(m1.diagonal(1).asDiagonal().toDenseMatrix(), m1.diagonalView(1).toDenseMatrix());

  // check equivalence to diagonal(i).asDiagonal() for compile time indexes
  VERIFY_IS_APPROX(m1.diagonal(0).asDiagonal().toDenseMatrix(), m1.template diagonalView<0>().toDenseMatrix());
  // sub
  VERIFY_IS_APPROX(m1.diagonal(-1).asDiagonal().toDenseMatrix(), m1.template diagonalView<-1>().toDenseMatrix());
  // super
  VERIFY_IS_APPROX(m1.diagonal(1).asDiagonal().toDenseMatrix(), m1.template diagonalView<1>().toDenseMatrix());

  // check const overloads
  const auto m2(m1);
  typedef decltype(m1) Type1;
  typedef decltype(m2) Type2;
  constexpr bool types_are_same = std::is_same<Type1, Type2>::value;
  VERIFY(!types_are_same);

  VERIFY_IS_APPROX(m2.diagonal(0).asDiagonal().toDenseMatrix(), m2.diagonalView(0).toDenseMatrix());
  VERIFY_IS_APPROX(m2.diagonal(1).asDiagonal().toDenseMatrix(), m2.template diagonalView<1>().toDenseMatrix());
}

EIGEN_DECLARE_TEST(diagonalview) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(diagonalview(Matrix<float, 3, 3>()));
    CALL_SUBTEST_2(diagonalview(Matrix<int, 50, 50>()));
  }
}
