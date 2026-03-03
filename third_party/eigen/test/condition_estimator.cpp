// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen (rmlarsen@gmail.com)
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"
#include <Eigen/Dense>

template <typename MatrixType>
typename MatrixType::RealScalar matrix_l1_norm(const MatrixType& m) {
  return m.cwiseAbs().colwise().sum().maxCoeff();
}

template <typename MatrixType>
void rcond_partial_piv_lu() {
  typedef typename MatrixType::RealScalar RealScalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);

  // Create a random diagonally dominant (thus invertible) matrix.
  MatrixType m = MatrixType::Random(size, size);
  m.diagonal().array() += RealScalar(2 * size);

  PartialPivLU<MatrixType> lu(m);
  MatrixType m_inverse = lu.inverse();
  RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m)) / matrix_l1_norm(m_inverse);
  RealScalar rcond_est = lu.rcond();
  // Verify the estimate is within a factor of 3 of the truth.
  VERIFY(rcond_est > rcond / 3 && rcond_est < rcond * 3);
}

template <typename MatrixType>
void rcond_full_piv_lu() {
  typedef typename MatrixType::RealScalar RealScalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);

  // Create a random diagonally dominant (thus invertible) matrix.
  MatrixType m = MatrixType::Random(size, size);
  m.diagonal().array() += RealScalar(2 * size);

  FullPivLU<MatrixType> lu(m);
  MatrixType m_inverse = lu.inverse();
  RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m)) / matrix_l1_norm(m_inverse);
  RealScalar rcond_est = lu.rcond();
  VERIFY(rcond_est > rcond / 3 && rcond_est < rcond * 3);
}

template <typename MatrixType>
void rcond_llt() {
  typedef typename MatrixType::RealScalar RealScalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);

  // Create a random SPD matrix: A^T * A + I.
  MatrixType a = MatrixType::Random(size, size);
  MatrixType m = a.adjoint() * a + MatrixType::Identity(size, size);

  LLT<MatrixType> llt(m);
  VERIFY(llt.info() == Success);
  MatrixType m_inverse = llt.solve(MatrixType::Identity(size, size));
  RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m)) / matrix_l1_norm(m_inverse);
  RealScalar rcond_est = llt.rcond();
  VERIFY(rcond_est > rcond / 3 && rcond_est < rcond * 3);
}

template <typename MatrixType>
void rcond_ldlt() {
  typedef typename MatrixType::RealScalar RealScalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);

  // Create a random SPD matrix: A^T * A + I.
  MatrixType a = MatrixType::Random(size, size);
  MatrixType m = a.adjoint() * a + MatrixType::Identity(size, size);

  LDLT<MatrixType> ldlt(m);
  VERIFY(ldlt.info() == Success);
  MatrixType m_inverse = ldlt.solve(MatrixType::Identity(size, size));
  RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m)) / matrix_l1_norm(m_inverse);
  RealScalar rcond_est = ldlt.rcond();
  VERIFY(rcond_est > rcond / 3 && rcond_est < rcond * 3);
}

template <typename MatrixType>
void rcond_singular() {
  typedef typename MatrixType::Scalar Scalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);

  // Create a rank-deficient matrix: first row is zero.
  MatrixType m = MatrixType::Random(size, size);
  m.row(0).setZero();

  FullPivLU<MatrixType> lu(m);
  VERIFY_IS_EQUAL(lu.rcond(), Scalar(0));
}

template <typename MatrixType>
void rcond_identity() {
  typedef typename MatrixType::RealScalar RealScalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(2, EIGEN_TEST_MAX_SIZE);

  MatrixType m = MatrixType::Identity(size, size);

  // All decompositions should give rcond ~= 1 for the identity.
  {
    PartialPivLU<MatrixType> lu(m);
    VERIFY(lu.rcond() > RealScalar(0.5));
  }
  {
    FullPivLU<MatrixType> lu(m);
    VERIFY(lu.rcond() > RealScalar(0.5));
  }
  {
    LLT<MatrixType> llt(m);
    VERIFY(llt.rcond() > RealScalar(0.5));
  }
  {
    LDLT<MatrixType> ldlt(m);
    VERIFY(ldlt.rcond() > RealScalar(0.5));
  }
}

template <typename MatrixType>
void rcond_ill_conditioned() {
  typedef typename MatrixType::RealScalar RealScalar;
  Index size = MatrixType::RowsAtCompileTime;
  if (size == Dynamic) size = internal::random<Index>(4, EIGEN_TEST_MAX_SIZE);

  // Create a diagonal matrix with known large condition number.
  // Use 1e-3 to stay well within single-precision range.
  MatrixType m = MatrixType::Zero(size, size);
  m(0, 0) = RealScalar(1);
  for (Index i = 1; i < size; ++i) {
    m(i, i) = RealScalar(1e-3);
  }
  // True condition number = 1e3, so rcond = 1e-3.

  {
    PartialPivLU<MatrixType> lu(m);
    RealScalar rcond_est = lu.rcond();
    VERIFY(rcond_est < RealScalar(1e-1));
    VERIFY(rcond_est > RealScalar(1e-5));
  }
  {
    FullPivLU<MatrixType> lu(m);
    RealScalar rcond_est = lu.rcond();
    VERIFY(rcond_est < RealScalar(1e-1));
    VERIFY(rcond_est > RealScalar(1e-5));
  }
}

template <typename MatrixType>
void rcond_1x1() {
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<typename MatrixType::Scalar, 1, 1> Mat1;
  Mat1 m;
  m(0, 0) = internal::random<RealScalar>(RealScalar(1), RealScalar(100));

  {
    PartialPivLU<Mat1> lu(m);
    VERIFY_IS_APPROX(lu.rcond(), RealScalar(1));
  }
  {
    FullPivLU<Mat1> lu(m);
    VERIFY_IS_APPROX(lu.rcond(), RealScalar(1));
  }
  {
    LLT<Mat1> llt(m);
    VERIFY_IS_APPROX(llt.rcond(), RealScalar(1));
  }
  {
    LDLT<Mat1> ldlt(m);
    VERIFY_IS_APPROX(ldlt.rcond(), RealScalar(1));
  }
}

template <typename MatrixType>
void rcond_2x2() {
  typedef typename MatrixType::RealScalar RealScalar;
  typedef Matrix<typename MatrixType::Scalar, 2, 2> Mat2;

  // Well-conditioned 2x2 matrix.
  Mat2 m;
  m << RealScalar(2), RealScalar(1), RealScalar(1), RealScalar(3);

  {
    PartialPivLU<Mat2> lu(m);
    Mat2 m_inverse = lu.inverse();
    RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m)) / matrix_l1_norm(m_inverse);
    RealScalar rcond_est = lu.rcond();
    VERIFY(rcond_est > rcond / 3 && rcond_est < rcond * 3);
  }
  {
    FullPivLU<Mat2> lu(m);
    Mat2 m_inverse = lu.inverse();
    RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m)) / matrix_l1_norm(m_inverse);
    RealScalar rcond_est = lu.rcond();
    VERIFY(rcond_est > rcond / 3 && rcond_est < rcond * 3);
  }
  {
    LLT<Mat2> llt(m);
    Mat2 m_inverse = llt.solve(Mat2::Identity());
    RealScalar rcond = (RealScalar(1) / matrix_l1_norm(m)) / matrix_l1_norm(m_inverse);
    RealScalar rcond_est = llt.rcond();
    VERIFY(rcond_est > rcond / 3 && rcond_est < rcond * 3);
  }
}

EIGEN_DECLARE_TEST(condition_estimator) {
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(rcond_partial_piv_lu<Matrix3f>());
    CALL_SUBTEST_1(rcond_full_piv_lu<Matrix3f>());
    CALL_SUBTEST_1(rcond_llt<Matrix3f>());
    CALL_SUBTEST_1(rcond_ldlt<Matrix3f>());
    CALL_SUBTEST_1(rcond_singular<Matrix3f>());
    CALL_SUBTEST_1(rcond_identity<Matrix3f>());
    CALL_SUBTEST_1(rcond_1x1<Matrix3f>());
    CALL_SUBTEST_1(rcond_2x2<Matrix3f>());

    CALL_SUBTEST_2(rcond_partial_piv_lu<Matrix4d>());
    CALL_SUBTEST_2(rcond_full_piv_lu<Matrix4d>());
    CALL_SUBTEST_2(rcond_llt<Matrix4d>());
    CALL_SUBTEST_2(rcond_ldlt<Matrix4d>());
    CALL_SUBTEST_2(rcond_singular<Matrix4d>());
    CALL_SUBTEST_2(rcond_identity<Matrix4d>());
    CALL_SUBTEST_2(rcond_2x2<Matrix4d>());

    CALL_SUBTEST_3(rcond_partial_piv_lu<MatrixXf>());
    CALL_SUBTEST_3(rcond_full_piv_lu<MatrixXf>());
    CALL_SUBTEST_3(rcond_llt<MatrixXf>());
    CALL_SUBTEST_3(rcond_ldlt<MatrixXf>());
    CALL_SUBTEST_3(rcond_singular<MatrixXf>());
    CALL_SUBTEST_3(rcond_identity<MatrixXf>());
    CALL_SUBTEST_3(rcond_ill_conditioned<MatrixXf>());

    CALL_SUBTEST_4(rcond_partial_piv_lu<MatrixXd>());
    CALL_SUBTEST_4(rcond_full_piv_lu<MatrixXd>());
    CALL_SUBTEST_4(rcond_llt<MatrixXd>());
    CALL_SUBTEST_4(rcond_ldlt<MatrixXd>());
    CALL_SUBTEST_4(rcond_singular<MatrixXd>());
    CALL_SUBTEST_4(rcond_identity<MatrixXd>());
    CALL_SUBTEST_4(rcond_ill_conditioned<MatrixXd>());

    CALL_SUBTEST_5(rcond_partial_piv_lu<MatrixXcf>());
    CALL_SUBTEST_5(rcond_full_piv_lu<MatrixXcf>());
    CALL_SUBTEST_5(rcond_llt<MatrixXcf>());
    CALL_SUBTEST_5(rcond_ldlt<MatrixXcf>());
    CALL_SUBTEST_5(rcond_singular<MatrixXcf>());
    CALL_SUBTEST_5(rcond_identity<MatrixXcf>());

    CALL_SUBTEST_6(rcond_partial_piv_lu<MatrixXcd>());
    CALL_SUBTEST_6(rcond_full_piv_lu<MatrixXcd>());
    CALL_SUBTEST_6(rcond_llt<MatrixXcd>());
    CALL_SUBTEST_6(rcond_ldlt<MatrixXcd>());
    CALL_SUBTEST_6(rcond_singular<MatrixXcd>());
    CALL_SUBTEST_6(rcond_identity<MatrixXcd>());
  }
}
