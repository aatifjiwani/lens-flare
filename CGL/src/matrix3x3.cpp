#include "matrix3x3.h"

#include <iostream>
#include <cmath>

using namespace std;

namespace CGL {

  double& Matrix3x3::operator()( int i, int j ) {
    return entries[j][i];
  }

  const double& Matrix3x3::operator()( int i, int j ) const {
    return entries[j][i];
  }

  Vector3D& Matrix3x3::operator[]( int j ) {
      return entries[j];
  }

  const Vector3D& Matrix3x3::operator[]( int j ) const {
    return entries[j];
  }

  void Matrix3x3::zero( double val ) {
    // sets all elements to val
    entries[0] = entries[1] = entries[2] = Vector3D( val, val, val );
  }

  double Matrix3x3::det( void ) const {
    const Matrix3x3& A( *this );

    return -A(0,2)*A(1,1)*A(2,0) + A(0,1)*A(1,2)*A(2,0) +
            A(0,2)*A(1,0)*A(2,1) - A(0,0)*A(1,2)*A(2,1) -
            A(0,1)*A(1,0)*A(2,2) + A(0,0)*A(1,1)*A(2,2) ;
  }

  double Matrix3x3::norm( void ) const {
    return sqrt( entries[0].norm2() +
                 entries[1].norm2() +
                 entries[2].norm2() );
  }

  Matrix3x3 Matrix3x3::operator-( void ) const {

   // returns -A
    const Matrix3x3& A( *this );
    Matrix3x3 B;

    B[0] = -A[0];
    B[1] = -A[1];
    B[2] = -A[2];

    return B;
  }

  void Matrix3x3::operator+=( const Matrix3x3& B ) {

    Matrix3x3& A( *this );

    A[0] += B[0];
    A[1] += B[1];
    A[2] += B[2];
  }

  Matrix3x3 Matrix3x3::operator-( const Matrix3x3& B ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 C;

    C[0] = A[0] - B[0];
    C[1] = A[1] - B[1];
    C[2] = A[2] - B[2];

    return C;
  }

  Matrix3x3 Matrix3x3::operator*( double c ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 B;

    B[0] = A[0] * c;
    B[1] = A[1] * c;
    B[2] = A[2] * c;

    return B;
  }

  Matrix3x3 operator*( double c, const Matrix3x3& A ) {
    Matrix3x3 cA;

    cA[0] = A[0] * c;
    cA[1] = A[1] * c;
    cA[2] = A[2] * c;

    return cA;
  }

  Matrix3x3 Matrix3x3::operator*( const Matrix3x3& B ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 C;

    C[0] = A * B[0];
    C[1] = A * B[1];
    C[2] = A * B[2];

    return C;
  }

  Vector3D Matrix3x3::operator*( const Vector3D& x ) const {
    return x.x * entries[0] +
           x.y * entries[1] +
           x.z * entries[2];
  }

  Matrix3x3 Matrix3x3::T( void ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 B;

    for( int i = 0; i < 3; i++ )
    for( int j = 0; j < 3; j++ )
    {
       B(i,j) = A(j,i);
    }

    return B;
  }

  Matrix3x3 Matrix3x3::inv( void ) const {
    const Matrix3x3& A( *this );
    Matrix3x3 B;

    B(0,0) = -A(1,2)*A(2,1) + A(1,1)*A(2,2); B(0,1) =  A(0,2)*A(2,1) - A(0,1)*A(2,2); B(0,2) = -A(0,2)*A(1,1) + A(0,1)*A(1,2);
    B(1,0) =  A(1,2)*A(2,0) - A(1,0)*A(2,2); B(1,1) = -A(0,2)*A(2,0) + A(0,0)*A(2,2); B(1,2) =  A(0,2)*A(1,0) - A(0,0)*A(1,2);
    B(2,0) = -A(1,1)*A(2,0) + A(1,0)*A(2,1); B(2,1) =  A(0,1)*A(2,0) - A(0,0)*A(2,1); B(2,2) = -A(0,1)*A(1,0) + A(0,0)*A(1,1);

    B /= det();

    return B;
  }

  void Matrix3x3::operator/=( double x ) {
    Matrix3x3& A( *this );
    double rx = 1./x;

    A[0] *= rx;
    A[1] *= rx;
    A[2] *= rx;
  }

  Matrix3x3 Matrix3x3::identity( void ) {
    Matrix3x3 B;

    B(0,0) = 1.; B(0,1) = 0.; B(0,2) = 0.;
    B(1,0) = 0.; B(1,1) = 1.; B(1,2) = 0.;
    B(2,0) = 0.; B(2,1) = 0.; B(2,2) = 1.;

    return B;
  }

  Matrix3x3 Matrix3x3::crossProduct( const Vector3D& u ) {
    Matrix3x3 B;

    B(0,0) =   0.;  B(0,1) = -u.z;  B(0,2) =  u.y;
    B(1,0) =  u.z;  B(1,1) =   0.;  B(1,2) = -u.x;
    B(2,0) = -u.y;  B(2,1) =  u.x;  B(2,2) =   0.;

    return B;
  }

  Matrix3x3 outer( const Vector3D& u, const Vector3D& v ) {
    Matrix3x3 B;
    double* Bij = (double*) &B;

    *Bij++ = u.x*v.x;
    *Bij++ = u.y*v.x;
    *Bij++ = u.z*v.x;
    *Bij++ = u.x*v.y;
    *Bij++ = u.y*v.y;
    *Bij++ = u.z*v.y;
    *Bij++ = u.x*v.z;
    *Bij++ = u.y*v.z;
    *Bij++ = u.z*v.z;

    return B;
  }

  std::ostream& operator<<( std::ostream& os, const Matrix3x3& A ) {
    for( int i = 0; i < 3; i++ )
    {
       os << "[ ";

       for( int j = 0; j < 3; j++ )
       {
          os << A(i,j) << " ";
       }

       os << "]" << std::endl;
    }

    return os;
  }

  Vector3D& Matrix3x3::column( int i ) {
    return entries[i];
  }

  const Vector3D& Matrix3x3::column( int i ) const {
    return entries[i];
  }
}
