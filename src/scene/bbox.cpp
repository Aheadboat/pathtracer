#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
	float t0x, t0y, t0z, t1x, t1y, t1z;
	t0x = (min.x - r.o.x) / r.d.x;
	t0y = (min.y - r.o.y) / r.d.y;
	t0z = (min.z - r.o.z) / r.d.z;
	t1x = (max.x - r.o.x) / r.d.x;
	t1y = (max.y - r.o.y) / r.d.y;
	t1z = (max.z - r.o.z) / r.d.z;
	if (t0x > t1x) {
		std::swap(t0x, t1x);
	}
	if (t0y > t1y) {
		std::swap(t0y, t1y);
	}
	if (t0z > t1z) {
		std::swap(t0z, t1z);
	}
	float tmin = std::max(std::max(t0x, t0y), t0z);
	float tmax = std::min(std::min(t1x, t1y), t1z);

	if (tmin > tmax || tmax < 0) {
		return false;
	}
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
