#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  // Setup quadratic equation
  double a = dot(r.d, r.d);
  double b = 2 * dot(r.o - this->o, r.d);
  double c = dot(r.o - this->o, r.o - this->o) - this->r2;

  if (b*b < 4.0 * a * c) {
    // there is no solution
    return false;
  } else if (b * b == 4.0 * a * c) {
    // there is only one root
    double possible_root = (-b) / (2.0 * a);
    if (possible_root < r.min_t || possible_root > r.max_t) {
      return false;
    }

    // set t1, t2, and ray.max_t
    t1 = t2 = possible_root;
    r.max_t = possible_root;
    return true;
  }

  double quadratic_term = sqrt(b * b - 4.0 * a * c);
  double root_1 = (-b - quadratic_term) / (2.0 * a);
  double root_2 = (-b + quadratic_term) / (2.0 * a);

  double poss_t1 = min(root_1, root_2);
  double poss_t2 = max(root_1, root_2);


  if (poss_t1 > r.max_t || poss_t2 < r.min_t) {
    // no possible valid intersection
    return false;
  }

  if (poss_t1 < r.min_t) {
    // poss_t2 is definitely not less than r.min_t
    // check if greater than max_t
    if (poss_t2 > r.max_t) {
      return false;
    }

    // intersection is with poss_t2
    t1 = t2 = poss_t2;
    return true;
  }

  // at this point, we know that poss_t1 is within bounds and is a valid intersection
  // it also must be <= poss_t2 so it must be the closest valid intersection as well
  // I don't think it matters at this point if poss_t2 > r.max_t
  t1 = poss_t1;
  t2 = poss_t2 > r.max_t ? poss_t1 : poss_t2;
  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  if (! test(r, t1, t2) ) {
    return false;
  }

//  r.max_t = t1;
  return true;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2;
  if (! test(r, t1, t2) ) {
    return false;
  }

  r.max_t = t1;

  // populate isect
  i->bsdf = this->get_bsdf();
  i->primitive = this;
  i->t = t1;

  // get normal vector and save to isect
  Vector3D isect_p = r.o + t1 * r.d;
  i->n = this->normal(isect_p);

  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
