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
  

  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

	// TODO (Part 1.4):
	// Implement ray - sphere intersection.
	// Note that you might want to use the the Sphere::test helper here.
	//o: origin, r: radius, r2: radius^2
	double a = dot(r.d, r.d);
	double b = 2 * dot(r.o - o, r.d);
	double c = dot(r.o - o, r.o - c) - r2;
	double inside = b * b - 4 * a * c;
	if (inside < 0) {
		return false;
	}
	else {
		inside = sqrt(inside);
	}
	double tmin = (-b - inside) / (2 * a);
	double tplus = (-b + inside) / (2 * a);
	if (abs(tmin) > abs(tplus)) {
		swap(tmin, tplus);
	}
	double t = tmin;
	if (t < r.min_t || t > r.max_t) {
		return false;
	}
	
	return true;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

	// TODO (Part 1.4):
	// Implement ray - sphere intersection.
	// Note again that you might want to use the the Sphere::test helper here.
	// When an intersection takes place, the Intersection data should be updated
	// correspondingly.
	double a = dot(r.d, r.d);
	double b = 2 * dot(r.o - o, r.d);
	double c = dot(r.o - o, r.o - o) - r2;
	double inside = b * b - 4 * a * c;
	if (inside < 0) {
		return false;
	}
	else {
		inside = sqrt(inside);
	}
	double tmin = (-b - inside) / (2 * a);
	double tplus = (-b + inside) / (2 * a);
	//if (abs(tmin) > abs(tplus)) {
	//	swap(tmin, tplus);
	//}
	double t;
	bool ret = false;
	if (tmin >= r.min_t && tmin <= r.max_t) {
		t = tmin;
		ret = true;			
	}
	else if (tplus >= r.min_t && tplus <= r.max_t) {
		t = tplus;
		ret = true;
	}
	if (t < r.min_t || t > r.max_t || !ret) {
		return false;
	}
	Vector3D intersect = r.o + t * r.d;
	Vector3D normal = intersect - o;
	normal.normalize();
	//update isect
	i->t = t;
	i->n = normal;
	i->primitive = this;
	i->bsdf = get_bsdf();
	r.max_t = t;
	
	
	
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
