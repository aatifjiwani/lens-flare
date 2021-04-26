#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>



namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
    
  // slab intervals:
  double tx1 = (this->min.x - r.o.x) / r.d.x;
  double tx2 = (this->max.x - r.o.x) / r.d.x;
    
  double ty1 = (this->min.y - r.o.y) / r.d.y;
  double ty2 = (this->max.y - r.o.y) / r.d.y;
    
  double tz1 = (this->min.z - r.o.z) / r.d.z;
  double tz2 = (this->max.z - r.o.z) / r.d.z;

  // max of mins, min of maxes
  double t_min = std::max({std::min(tx1, tx2),std::min(ty1, ty2),std::min(tz1, tz2)});
  double t_max = std::min({std::max(tx1, tx2),std::max(ty1, ty2),std::max(tz1, tz2)});
    
  // if min > max
  if (t_min > t_max) {
      return false;
  }

  if (t_max < t0 || t_min > t1) {
    return false;
  }

  t0 = std::max(t0, t_min);
  t1 = std::min(t1, t_max);

  t0 = t_min;
  t1 = t_max;

  return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
