#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

Vector3D moller_trumbore(const Vector3D &p0, const Vector3D &p1, const Vector3D &p2, const Ray &r) {
    Vector3D e1, e2, s, s1, s2;

    e1 = p1 - p0;
    e2 = p2 - p0;
    s = r.o - p0;

    s1 = cross(r.d, e2);
    s2 = cross(s, e1);

    Vector3D t_b1_b2 = Vector3D();
    t_b1_b2.x = dot(s2,  e2);
    t_b1_b2.y = dot(s1,   s);
    t_b1_b2.z = dot(s2, r.d);

    t_b1_b2 /= dot(s1, e1);

    return t_b1_b2;
}

bool is_valid_intersection(Vector3D &t_b1_b2, const Ray &r) {
    double t = t_b1_b2.x;

    if (t < r.min_t || t > r.max_t) {
        return false;
    }

    double b1 = t_b1_b2.y;
    double b2 = t_b1_b2.z;

    if (b1 < 0 || b1 > 1) {
        return false;
    } else if (b2 < 0 || b2 > 1) {
        return false;
    }

    // at this point, we know 0 <= b1 <= 1 and 0 <= b2 <= 1
    // now need to test validity of b0
    if (b1 + b2 > 1) {
        return false;
    }

    return true;
}

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
  Vector3D t_b1_b2 = moller_trumbore(p1, p2, p3, r);
  if ( ! is_valid_intersection(t_b1_b2, r)) {
      return false;
  }

  // set new max_t
//  r.max_t = t_b1_b2.x;
  return true;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D t_b1_b2 = moller_trumbore(p1, p2, p3, r);
  if ( ! is_valid_intersection(t_b1_b2, r)) {
      return false;
  }

  double t = t_b1_b2.x;
  double b1 = t_b1_b2.y;
  double b2 = t_b1_b2.z;
  double b0 = 1 - b1 - b2;

  // set new max_t
  r.max_t = t;

  // Populate isect
  isect->t = t;
  isect->primitive = this;
  isect->bsdf = this->get_bsdf();

  Vector3D interp_n = b0 * n1 + b1 * n2 + b2 * n3;
  isect->n = interp_n.unit();

  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
