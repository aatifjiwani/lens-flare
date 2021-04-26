#ifndef CGL_VECTOR4D_H
#define CGL_VECTOR4D_H

#include "CGL.h"
#include "vector3D.h"

#include <ostream>
#include <cmath>

namespace CGL {

/**
 * Defines 4D standard vectors.
 */
class Vector4D {
public:

  // components
  union {
    struct {
      double x, y, z, w;
    };
    struct {
      double r, g, b, a;
    };
#ifdef __AVX__
    struct {
      __m128d xy, zw;
    };
#endif
  };

  /**
   * Constructor.
   * Initializes tp vector (0,0,0, 0).
   */
  Vector4D() : x( 0.0 ), y( 0.0 ), z( 0.0 ), w( 0.0 ) { }

  /**
   * Constructor.
   * Initializes to vector (x,y,z,w).
   */
  Vector4D( double x, double y, double z, double w) : x( x ), y( y ), z( z ), w( w ) { }

  /**
   * Constructor.
   * Initializes to vector (x,y,z,0).
   */
  Vector4D( double x, double y, double z) : x( x ), y( y ), z( z ), w( 0.0 ) { }


  /**
   * Constructor.
   * Initializes to vector (c,c,c,c)
   */
#ifdef __AVX__
  Vector4D(double c) { xy = _mm_set1_pd(c); zw = _mm_set1_pd(1); }
#else
  Vector4D(double c) : x(c), y(c), z(c), w(c) {}
#endif

  /**
   * Constructor.
   * Initializes from existing vector4D.
   */
  Vector4D( const Vector4D& v ) : x( v.x ), y( v.y ), z( v.z ), w( v.w ) { }

  /**
   * Constructor.
   * Initializes from existing vector3D.
   */
  Vector4D( const Vector3D& v ) : x( v.x ), y( v.y ), z( v.z ), w( 0.0 ) { }

#ifdef __AVX__
  Vector4D(__m128d a, __m128d b) : xy(a), zw(b) {}
#endif

  /**
   * Constructor.
   * Initializes from existing vector3D and w value.
   */
  Vector4D( const Vector3D& v, double w ) : x( v.x ), y( v.y ), z( v.z ), w( w ) { }

  // returns reference to the specified component (0-based indexing: x, y, z)
  inline double& operator[] ( const int& index ) {
    return ( &x )[ index ];
  }

  // returns const reference to the specified component (0-based indexing: x, y, z)
  inline const double& operator[] ( const int& index ) const {
    return ( &x )[ index ];
  }

  // negation
  inline Vector4D operator-( void ) const {
#ifdef __AVX__
    return Vector4D(_mm_sub_pd(_mm_set1_pd(0.0), xy), _mm_sub_pd(_mm_set1_pd(0.0), zw));
#else
    return Vector4D(-x, -y, -z, -w);
#endif
  }

  // addition
  inline Vector4D operator+( const Vector4D& v ) const {
#ifdef __AVX__
    return Vector4D(_mm_add_pd(xy, v.xy), _mm_add_pd(zw, v.zw));
#else
    return Vector4D(x + v.x, y + v.y, z + v.z, w + v.w);
#endif
  }

  // subtraction
  inline Vector4D operator-( const Vector4D& v ) const {
#ifdef __AVX__
    return Vector4D(_mm_sub_pd(xy, v.xy), _mm_sub_pd(zw, v.zw));
#else
    return Vector4D(x - v.x, y - v.y, z - v.z, w - v.w);
#endif
  }

  // right scalar multiplication
  inline Vector4D operator*( const double& c ) const {
#ifdef __AVX__
    __m128d cv = _mm_set1_pd(c);
    return Vector4D(_mm_mul_pd(xy, cv), _mm_mul_pd(zw, cv));
#else
    return Vector4D(x * c, y * c, z * c, w * c);
#endif
  }

  // scalar division
  inline Vector4D operator/( const double& c ) const {
    const double rc = 1.0/c;
#ifdef __AVX__
    __m128d rcv = _mm_set1_pd(rc);
    return Vector4D(_mm_mul_pd(xy, rcv), _mm_mul_pd(zw, rcv));
#else
    return Vector4D(rc * x, rc * y, rc * z, rc * w);
#endif
  }

  // addition / assignment
  inline void operator+=( const Vector4D& v ) {
#ifdef __AVX__
    xy = _mm_add_pd(xy, v.xy);
    zw = _mm_add_pd(zw, v.zw);
#else
    x += v.x;
    y += v.y;
    z += v.z;
    z += v.w;
#endif
  }

  // subtraction / assignment
  inline void operator-=( const Vector4D& v ) {
#ifdef __AVX__
    xy = _mm_sub_pd(xy, v.xy);
    zw = _mm_sub_pd(zw, v.zw);
#else
    x -= v.x;
    y -= v.y;
    z -= v.z;
    w -= v.w;
#endif
  }

  // scalar multiplication / assignment
  inline void operator*=( const double& c ) {
#ifdef __AVX__
    __m128d cv = _mm_set1_pd(c);
    xy = _mm_add_pd(xy, cv);
    zw = _mm_add_pd(zw, cv);
#else
    x *= c;
    y *= c;
    z *= c;
    w *= c;
#endif
  }

  // scalar division / assignment
  inline void operator/=( const double& c ) {
#ifdef __AVX__
    __m128d cv = _mm_set1_pd(c);
    xy = _mm_div_pd(xy, cv);
    zw = _mm_div_pd(zw, cv);
#else
    (*this) *= (1. / c);
#endif
  }

  /**
   * Returns per entry reciprocal
   */
  inline Vector4D rcp(void) const {
#ifdef __AVX__
    return Vector4D(_mm_div_pd(_mm_set1_pd(1.0), xy),
                    _mm_div_pd(_mm_set1_pd(1.0), zw));
#else
    return Vector4D(1.0 / x, 1.0 / y, 1.0 / z, 1.0 / w);
#endif
  }

  /**
   * Returns Euclidean distance metric extended to 4 dimensions.
   */
  inline double norm( void ) const {
    return sqrt(norm2());
  }

  /**
   * Returns Euclidean length squared.
   */
  inline double norm2( void ) const {
#ifdef __AVX__
    return _mm_cvtsd_f64(_mm_add_sd(_mm_dp_pd(xy, xy, 0b00110001), _mm_dp_pd(zw, zw, 0b00110001)));
#else
    return x*x + y*y + z*z + w*w;
#endif
  }

  /**
   * Returns unit vector. (returns the normalized copy of this vector.)
   */
  inline Vector4D unit( void ) const {
    double rNorm = 1. / sqrt( x*x + y*y + z*z + w*w);
    return Vector4D( rNorm*x, rNorm*y, rNorm*z );
  }

  /**
   * Divides by Euclidean length.
   * This vector will be of unit length i.e. "normalized" afterwards.
   */
  inline void normalize( void ) {
    (*this) /= norm();
  }

  /**
   * Converts this vector to a 3D vector ignoring the w component.
   */
  Vector3D to3D();

  /**
   * Converts this vector to a 3D vector by dividing x, y, and z by w.
   */
  Vector3D projectTo3D();

}; // class Vector4D

// left scalar multiplication
inline Vector4D operator* ( const double& c, const Vector4D& v ) {
  return Vector4D( c * v.x, c * v.y, c * v.z, c*v.w );
}

// dot product (a.k.a. inner or scalar product)
inline double dot( const Vector4D& u, const Vector4D& v ) {
  return u.x*v.x + u.y*v.y + u.z*v.z + u.w*v.w;;
}

// prints components
std::ostream& operator<<( std::ostream& os, const Vector4D& v );

} // namespace CGL

#endif // CGL_VECTOR3D_H
