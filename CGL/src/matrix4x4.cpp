#include "matrix4x4.h"

#include <cmath>
#include <iostream>


using namespace std;

namespace CGL {

double &Matrix4x4::operator()(int i, int j) { return entries[j][i]; }

const double &Matrix4x4::operator()(int i, int j) const {
  return entries[j][i];
}

Vector4D &Matrix4x4::operator[](int j) { return entries[j]; }

const Vector4D &Matrix4x4::operator[](int j) const { return entries[j]; }

void Matrix4x4::zero(double val) {
  // sets all elements to val
  entries[0] = Vector4D(val);
  entries[1] = Vector4D(val);
  entries[2] = Vector4D(val);
  entries[3] = Vector4D(val);
}

double Matrix4x4::det(void) const {
  const Matrix4x4 &A(*this);

  return A(0, 3) * A(1, 2) * A(2, 1) * A(3, 0) -
         A(0, 2) * A(1, 3) * A(2, 1) * A(3, 0) -
         A(0, 3) * A(1, 1) * A(2, 2) * A(3, 0) +
         A(0, 1) * A(1, 3) * A(2, 2) * A(3, 0) +
         A(0, 2) * A(1, 1) * A(2, 3) * A(3, 0) -
         A(0, 1) * A(1, 2) * A(2, 3) * A(3, 0) -
         A(0, 3) * A(1, 2) * A(2, 0) * A(3, 1) +
         A(0, 2) * A(1, 3) * A(2, 0) * A(3, 1) +
         A(0, 3) * A(1, 0) * A(2, 2) * A(3, 1) -
         A(0, 0) * A(1, 3) * A(2, 2) * A(3, 1) -
         A(0, 2) * A(1, 0) * A(2, 3) * A(3, 1) +
         A(0, 0) * A(1, 2) * A(2, 3) * A(3, 1) +
         A(0, 3) * A(1, 1) * A(2, 0) * A(3, 2) -
         A(0, 1) * A(1, 3) * A(2, 0) * A(3, 2) -
         A(0, 3) * A(1, 0) * A(2, 1) * A(3, 2) +
         A(0, 0) * A(1, 3) * A(2, 1) * A(3, 2) +
         A(0, 1) * A(1, 0) * A(2, 3) * A(3, 2) -
         A(0, 0) * A(1, 1) * A(2, 3) * A(3, 2) -
         A(0, 2) * A(1, 1) * A(2, 0) * A(3, 3) +
         A(0, 1) * A(1, 2) * A(2, 0) * A(3, 3) +
         A(0, 2) * A(1, 0) * A(2, 1) * A(3, 3) -
         A(0, 0) * A(1, 2) * A(2, 1) * A(3, 3) -
         A(0, 1) * A(1, 0) * A(2, 2) * A(3, 3) +
         A(0, 0) * A(1, 1) * A(2, 2) * A(3, 3);
}

double Matrix4x4::norm(void) const {
  return sqrt(entries[0].norm2() + entries[1].norm2() + entries[2].norm2() +
              entries[3].norm2());
}

Matrix4x4 Matrix4x4::operator-(void) const {
  // returns -A (Negation).
  const Matrix4x4 &A(*this);
  Matrix4x4 B;

  B[0] = -A[0];
  B[1] = -A[1];
  B[2] = -A[2];
  B[3] = -A[3];

  return B;
}

void Matrix4x4::operator+=(const Matrix4x4 &B) {

  Matrix4x4 &A(*this);
  double *Aij = (double *)&A;
  const double *Bij = (const double *)&B;

  A[0] += B[0];
  A[1] += B[1];
  A[2] += B[2];
  A[3] += B[3];
}

Matrix4x4 Matrix4x4::operator-(const Matrix4x4 &B) const {
  const Matrix4x4 &A(*this);
  Matrix4x4 C;

  C[0] = A[0] - B[0];
  C[1] = A[1] - B[1];
  C[2] = A[2] - B[2];
  C[3] = A[3] - B[3];

  return C;
}

Matrix4x4 Matrix4x4::operator*(double c) const {
  const Matrix4x4 &A(*this);
  Matrix4x4 B;

  B[0] = c * A[0];
  B[1] = c * A[1];
  B[2] = c * A[2];
  B[3] = c * A[3];

  return B;
}

// Returns c*A.
Matrix4x4 operator*(double c, const Matrix4x4 &A) {
  Matrix4x4 cA;

  cA[0] = c * A[0];
  cA[1] = c * A[1];
  cA[2] = c * A[2];
  cA[3] = c * A[3];

  return cA;
}

// Tradiational Grade School Multiplication. N^3
Matrix4x4 Matrix4x4::operator*(const Matrix4x4 &B) const {
  const Matrix4x4 &A(*this);
  Matrix4x4 C;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
#ifdef __AVX__
      C(i, j) = dot(A[i], Vector4D(B(0, j), B(1, j), B(2, j), B(3, j)));
#else
      C(i, j) = 0.;

      for (int k = 0; k < 4; k++) {
        C(i, j) += A(i, k) * B(k, j);
      }
#endif
    }
  }

  return C;
}

Vector4D Matrix4x4::operator*(const Vector4D &x) const {
  return x[0] * entries[0] + // Add up products for each matrix column.
         x[1] * entries[1] + x[2] * entries[2] + x[3] * entries[3];
}

// Naive Transposition.
Matrix4x4 Matrix4x4::T(void) const {
  const Matrix4x4 &A(*this);
  Matrix4x4 B;

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      B(i, j) = A(j, i);
    }

  return B;
}

Matrix4x4 Matrix4x4::inv(void) const {
  const Matrix4x4 &A(*this);
  Matrix4x4 B;

  // Hardcoded in Fully Symbolic computation.

  B(0, 0) = A(1, 2) * A(2, 3) * A(3, 1) - A(1, 3) * A(2, 2) * A(3, 1) +
            A(1, 3) * A(2, 1) * A(3, 2) - A(1, 1) * A(2, 3) * A(3, 2) -
            A(1, 2) * A(2, 1) * A(3, 3) + A(1, 1) * A(2, 2) * A(3, 3);
  B(0, 1) = A(0, 3) * A(2, 2) * A(3, 1) - A(0, 2) * A(2, 3) * A(3, 1) -
            A(0, 3) * A(2, 1) * A(3, 2) + A(0, 1) * A(2, 3) * A(3, 2) +
            A(0, 2) * A(2, 1) * A(3, 3) - A(0, 1) * A(2, 2) * A(3, 3);
  B(0, 2) = A(0, 2) * A(1, 3) * A(3, 1) - A(0, 3) * A(1, 2) * A(3, 1) +
            A(0, 3) * A(1, 1) * A(3, 2) - A(0, 1) * A(1, 3) * A(3, 2) -
            A(0, 2) * A(1, 1) * A(3, 3) + A(0, 1) * A(1, 2) * A(3, 3);
  B(0, 3) = A(0, 3) * A(1, 2) * A(2, 1) - A(0, 2) * A(1, 3) * A(2, 1) -
            A(0, 3) * A(1, 1) * A(2, 2) + A(0, 1) * A(1, 3) * A(2, 2) +
            A(0, 2) * A(1, 1) * A(2, 3) - A(0, 1) * A(1, 2) * A(2, 3);
  B(1, 0) = A(1, 3) * A(2, 2) * A(3, 0) - A(1, 2) * A(2, 3) * A(3, 0) -
            A(1, 3) * A(2, 0) * A(3, 2) + A(1, 0) * A(2, 3) * A(3, 2) +
            A(1, 2) * A(2, 0) * A(3, 3) - A(1, 0) * A(2, 2) * A(3, 3);
  B(1, 1) = A(0, 2) * A(2, 3) * A(3, 0) - A(0, 3) * A(2, 2) * A(3, 0) +
            A(0, 3) * A(2, 0) * A(3, 2) - A(0, 0) * A(2, 3) * A(3, 2) -
            A(0, 2) * A(2, 0) * A(3, 3) + A(0, 0) * A(2, 2) * A(3, 3);
  B(1, 2) = A(0, 3) * A(1, 2) * A(3, 0) - A(0, 2) * A(1, 3) * A(3, 0) -
            A(0, 3) * A(1, 0) * A(3, 2) + A(0, 0) * A(1, 3) * A(3, 2) +
            A(0, 2) * A(1, 0) * A(3, 3) - A(0, 0) * A(1, 2) * A(3, 3);
  B(1, 3) = A(0, 2) * A(1, 3) * A(2, 0) - A(0, 3) * A(1, 2) * A(2, 0) +
            A(0, 3) * A(1, 0) * A(2, 2) - A(0, 0) * A(1, 3) * A(2, 2) -
            A(0, 2) * A(1, 0) * A(2, 3) + A(0, 0) * A(1, 2) * A(2, 3);
  B(2, 0) = A(1, 1) * A(2, 3) * A(3, 0) - A(1, 3) * A(2, 1) * A(3, 0) +
            A(1, 3) * A(2, 0) * A(3, 1) - A(1, 0) * A(2, 3) * A(3, 1) -
            A(1, 1) * A(2, 0) * A(3, 3) + A(1, 0) * A(2, 1) * A(3, 3);
  B(2, 1) = A(0, 3) * A(2, 1) * A(3, 0) - A(0, 1) * A(2, 3) * A(3, 0) -
            A(0, 3) * A(2, 0) * A(3, 1) + A(0, 0) * A(2, 3) * A(3, 1) +
            A(0, 1) * A(2, 0) * A(3, 3) - A(0, 0) * A(2, 1) * A(3, 3);
  B(2, 2) = A(0, 1) * A(1, 3) * A(3, 0) - A(0, 3) * A(1, 1) * A(3, 0) +
            A(0, 3) * A(1, 0) * A(3, 1) - A(0, 0) * A(1, 3) * A(3, 1) -
            A(0, 1) * A(1, 0) * A(3, 3) + A(0, 0) * A(1, 1) * A(3, 3);
  B(2, 3) = A(0, 3) * A(1, 1) * A(2, 0) - A(0, 1) * A(1, 3) * A(2, 0) -
            A(0, 3) * A(1, 0) * A(2, 1) + A(0, 0) * A(1, 3) * A(2, 1) +
            A(0, 1) * A(1, 0) * A(2, 3) - A(0, 0) * A(1, 1) * A(2, 3);
  B(3, 0) = A(1, 2) * A(2, 1) * A(3, 0) - A(1, 1) * A(2, 2) * A(3, 0) -
            A(1, 2) * A(2, 0) * A(3, 1) + A(1, 0) * A(2, 2) * A(3, 1) +
            A(1, 1) * A(2, 0) * A(3, 2) - A(1, 0) * A(2, 1) * A(3, 2);
  B(3, 1) = A(0, 1) * A(2, 2) * A(3, 0) - A(0, 2) * A(2, 1) * A(3, 0) +
            A(0, 2) * A(2, 0) * A(3, 1) - A(0, 0) * A(2, 2) * A(3, 1) -
            A(0, 1) * A(2, 0) * A(3, 2) + A(0, 0) * A(2, 1) * A(3, 2);
  B(3, 2) = A(0, 2) * A(1, 1) * A(3, 0) - A(0, 1) * A(1, 2) * A(3, 0) -
            A(0, 2) * A(1, 0) * A(3, 1) + A(0, 0) * A(1, 2) * A(3, 1) +
            A(0, 1) * A(1, 0) * A(3, 2) - A(0, 0) * A(1, 1) * A(3, 2);
  B(3, 3) = A(0, 1) * A(1, 2) * A(2, 0) - A(0, 2) * A(1, 1) * A(2, 0) +
            A(0, 2) * A(1, 0) * A(2, 1) - A(0, 0) * A(1, 2) * A(2, 1) -
            A(0, 1) * A(1, 0) * A(2, 2) + A(0, 0) * A(1, 1) * A(2, 2);

  // Invertable iff the determinant is not equal to zero.
  B /= det();

  return B;
}

void Matrix4x4::operator/=(double x) {
  Matrix4x4 &A(*this);
  double rx = 1. / x;

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      A(i, j) *= rx;
    }
}

Matrix4x4 Matrix4x4::identity(void) {
  Matrix4x4 B;

  B(0, 0) = 1.;
  B(0, 1) = 0.;
  B(0, 2) = 0.;
  B(0, 3) = 0.;
  B(1, 0) = 0.;
  B(1, 1) = 1.;
  B(1, 2) = 0.;
  B(1, 3) = 0.;
  B(2, 0) = 0.;
  B(2, 1) = 0.;
  B(2, 2) = 1.;
  B(2, 3) = 0.;
  B(3, 0) = 0.;
  B(3, 1) = 0.;
  B(3, 2) = 0.;
  B(3, 3) = 1.;

  return B;
}

Matrix4x4 outer(const Vector4D &u, const Vector4D &v) {
  Matrix4x4 B;

  // Opposite of an inner product.
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      B(i, j) = u[i] * v[j];
    }

  return B;
}

std::ostream &operator<<(std::ostream &os, const Matrix4x4 &A) {
  for (int i = 0; i < 4; i++) {
    os << "[ ";

    for (int j = 0; j < 4; j++) {
      os << A(i, j) << " ";
    }

    os << "]" << std::endl;
  }

  return os;
}

Vector4D &Matrix4x4::column(int i) { return entries[i]; }

const Vector4D &Matrix4x4::column(int i) const { return entries[i]; }
} // namespace CGL
