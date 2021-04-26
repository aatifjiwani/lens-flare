#ifndef CGL_RANDOMUTIL_H
#define CGL_RANDOMUTIL_H

#include <random>

// #define XORSHIFT_RAND

namespace CGL {

static std::mersenne_twister_engine<std::uint_fast32_t, 32, 624, 397, 31, 0x9908b0df,
                             11, 0xffffffff, 7, 0x9d2c5680, 15, 0xefc60000, 18,
                             1812433253> minstd_engine;

static double rmax = 1.0 / (minstd_engine.max() - minstd_engine.min());


/**
 * Returns a number distributed uniformly over [0, 1].
 */
inline double random_uniform() {
  return clamp(double(minstd_engine() - minstd_engine.min()) * rmax, 0.0000001, 0.99999999);
}

/**
 * Returns true with probability p and false with probability 1 - p.
 */
inline bool coin_flip(double p) {
  return random_uniform() < p;
}

} // namespace CGL

#endif  // CGL_RANDOMUTIL_H
