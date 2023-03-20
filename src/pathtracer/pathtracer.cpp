#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray& r,
    const Intersection& isect) {
    // Estimate the lighting from this intersection coming directly from a light.
    // For this function, sample uniformly in a hemisphere.

    // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
    // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w; //object 2 world space
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T(); //world 2 object space

    // w_out points towards the source of the ray (e.g.,
    // toward the camera if this is a primary ray)
    const Vector3D hit_p = r.o + r.d * isect.t; //hit point of ray (world space)
    const Vector3D w_out = w2o * (-r.d);

    // This is the same number of total samples as
    // estimate_direct_lighting_importance (outside of delta lights). We keep the
    // same number of samples for clarity of comparison.
    int num_samples = scene->lights.size() * ns_area_light;
    Vector3D L_out;

    auto bsdf = isect.bsdf;
    // TODO (Part 3): Write your sampling loop here
    // TODO BEFORE YOU BEGIN
    // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
    //Monte-Carlo estimate
    for (int i = 0; i < num_samples; i++) {
        Intersection new_isect;
        /*Vector3D s_d = hemisphereSampler->get_sample();
        s_d.normalize();*/
        Vector3D s_d = hemisphereSampler->get_sample();
        double pdf = 1 / PI;
        Vector3D f = isect.bsdf->f(w_out, s_d);
        Vector3D s_d_w = o2w * s_d;
        Ray rand_ray(hit_p, s_d_w, 1);
        double cosTheta = cos_theta(s_d);
        //check if ray intersects, if not, then we outta here
        rand_ray.min_t = EPS_F;
        //Lambert's Law
        if (bvh->intersect(rand_ray, &new_isect) && cosTheta > 0) {
            L_out += (new_isect.bsdf->get_emission()) * f * cosTheta / pdf;
        }
    }



    return L_out / (double)(num_samples);

}

    



Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
  int M = scene->lights.size();
  Vector3D wi;
  Vector3D radiance;
  //Vector3D wi_w;
  double distToLight;
  double pdf;
  double cosTheta;
  Ray shadow_ray;
  for (auto light : scene->lights) {
      // sample only once for point lights
      if (light->is_delta_light()) {
          Intersection new_isect;
          radiance = light->sample_L(hit_p, &wi, &distToLight, &pdf);
          shadow_ray = Ray(hit_p, wi, 1);
          shadow_ray.min_t = EPS_F;
          shadow_ray.max_t = distToLight - EPS_F;
          cosTheta = dot(wi, isect.n);
          if (!bvh->intersect(shadow_ray, &new_isect) && cosTheta > 0) {
              L_out += radiance * isect.bsdf->f(w_out, wi) * cosTheta / pdf;
          }
      }
      // else we have to sample a bunch
      else {
          Vector3D L = Vector3D(0, 0, 0);
          for (int i = 0; i < ns_area_light; i++) {
            Intersection new_isect;
            radiance = light->sample_L(hit_p, &wi, &distToLight, &pdf);
            shadow_ray = Ray(hit_p, wi);
            shadow_ray.min_t = EPS_F; 
            shadow_ray.max_t = distToLight - EPS_F;
            cosTheta = dot(wi, isect.n);
            if (!bvh->intersect(shadow_ray, &new_isect) && cosTheta > 0) {
                L_out += ((radiance * isect.bsdf->f(w_out, wi) * cosTheta) / pdf) / ns_area_light;
            }
              
          }

      }
  }

  return L_out;

}
Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
    return isect.bsdf->get_emission();

}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

    if (direct_hemisphere_sample) {
        return estimate_direct_lighting_hemisphere(r, isect);
    }
    return estimate_direct_lighting_importance(r, isect);


}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
    L_out += one_bounce_radiance(r, isect);
  

  if (r.depth < 1) {
      return L_out;
  }

  Vector3D wi;
  double pdf;
  Vector3D f = isect.bsdf->sample_f(w_out, &wi, &pdf);
  Intersection new_isect = Intersection(isect);
  Vector3D wi_w = o2w * wi;
  double cosTheta = cos_theta(wi);
  double prob = 0.35;
  if (coin_flip(1.0-prob)) {
      Ray bounce = Ray(hit_p, wi_w, (int) (r.depth - 1));
      bounce.min_t = EPS_F;
      //bounce.max_t = 
      if (bvh->intersect(bounce, &new_isect) && cosTheta > 0) {
          L_out += ((at_least_one_bounce_radiance(bounce, new_isect)
              * f * cosTheta) / pdf) / (1.0 - prob);
      }

            
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.

  if (!bvh->intersect(r, &isect)) {
      //return envLight ? envLight->sample_dir(r) : L_out;
      return L_out;
  }

  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  //// TODO (Part 3): Return the direct illumination.
  L_out = Vector3D(0, 0, 0);
  L_out += zero_bounce_radiance(r, isect);
  L_out += at_least_one_bounce_radiance(r, isect);


  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  double num_samples = 0;          // total samples evaluated
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  double xk = 0;
  double xk_squared = 0;
  //sample ns_aa times
  Vector3D integral;
  bool cont = true;
  for (int i = 0; i < ns_aa && cont; i++) {
      if (i % samplesPerBatch == 0 && i != 0) {
          float variance = (1 / (num_samples - 1)) * (xk_squared - (xk * xk) / num_samples);
          float I = 1.96 * (sqrt(variance) / sqrt(num_samples));
          if (I <= maxTolerance * (xk / num_samples)) {
              cont = false;
          }
      }
      Vector2D samp = gridSampler->get_sample();
      double normalized_x = (x + samp.x) / sampleBuffer.w;
      double normalized_y = (y + samp.y) / sampleBuffer.h;
      Ray sample_ray = camera->generate_ray(normalized_x, normalized_y);
      sample_ray.depth = max_ray_depth;
      Vector3D sample_color = est_radiance_global_illumination(sample_ray);
      xk += sample_color.illum();
      xk_squared += sample_color.illum() * sample_color.illum();
      integral += sample_color;
      num_samples++;

  }
  integral /= (double) num_samples;
  sampleBuffer.update_pixel(integral, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
