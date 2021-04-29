#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"

#include <cstdlib>

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

void PathTracer::find_sun_pos() {
  cout << "have num lights " << scene->lights.size() << endl;
  for (SceneLight* light : scene->lights) {
    if (dynamic_cast<const DirectionalLight*>(light) != NULL) {
      DirectionalLight* dlight = (DirectionalLight*) light;
      std::cout << "Analyzing Directional Light with world coords: " << dlight->posLight << std::endl;
      double ns_x, ns_y;
      camera->analyze_world_coord(dlight->posLight, ns_x, ns_y);

      if ((ns_x >= 0 && ns_x <= 1) && (ns_y >= 0 && ns_y <=1)) {
        flare_origins.emplace_back(ns_x, ns_y);
        flare_radiance.push_back(dlight->radiance);
      }
    }
  }

  for (int l = 0; l < flare_origins.size(); l++ ) {
    cout << "found directional light at norm image coords: " << flare_origins[l] << " w radiance " << flare_radiance[l] << endl;
  }
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
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out = Vector3D(0.0);

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading

  double p_w = 1.0 / (2.0 * PI);
  for (int i = 0; i < num_samples; i++) {
    Vector3D incoming_wi = hemisphereSampler->get_sample(); // Probability 1/2*PI
    Vector3D incoming_wi_world = o2w * incoming_wi;

    Intersection outgoing_isect;
    // Need to offset hit_p such that the hit_p is not below the surface we intersected
    Ray outgoing_ray = Ray(hit_p, incoming_wi_world);
    outgoing_ray.min_t = EPS_F;

    if (bvh->intersect(outgoing_ray, &outgoing_isect)) {
      double cos_theta_out = dot(Vector3D(0,0,1), incoming_wi.unit());

      // incoming_wi is pointing outward, change direction to be inward.
      Vector3D ratio = isect.bsdf->f(-1 * incoming_wi, w_out); // need to use current isect
      Vector3D emission = outgoing_isect.bsdf->get_emission(); // need to use new intersection isect

      L_out += (ratio * emission * cos_theta_out) / p_w;

    }
  }

  return L_out / (double) num_samples;

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
  Vector3D L_out = Vector3D(0);

  int total_samples = 0;

  for (SceneLight* light : scene->lights) {
    // Calculate the number of samples to take
    // For point light - 1 sample
    // For area light - ns_area_light
    int num_samples;
    if (light->is_delta_light()) {
      num_samples = 1;
    } else {
      num_samples = ns_area_light;
    }
    total_samples += num_samples;

    for (int i = 0; i < num_samples; i++) {
      // Sample direction from light
      Vector3D incoming_wi_world;
      double dist_to_light;
      double pdf;

      Vector3D emission = light->sample_L(hit_p, &incoming_wi_world, &dist_to_light, &pdf);
      Vector3D incoming_wi_object = w2o * incoming_wi_world;

      // Check if light is behind us using dot products between
      // Normal vector (0, 0, 1) and object coord incoming direction
      if (dot(incoming_wi_object, Vector3D(0,0,1)) < 0) {
        continue;
      }

      // Cast the ray with origin hit_p, direction incoming_wi_world
      // min_t EPS_F, max_t dist_to_light - EPS_F
      Intersection light_isect;
      Ray outgoing_ray = Ray(hit_p, incoming_wi_world);
      outgoing_ray.min_t = EPS_F;
      outgoing_ray.max_t = dist_to_light - EPS_F;

      // Check if intersection occurs. If it DOES NOT, then the ray hits the light
      // source.
      if (!bvh->intersect(outgoing_ray, &light_isect)) {
        double cos_theta_out = dot(incoming_wi_object.unit(), Vector3D(0,0,1));
        Vector3D ratio = isect.bsdf->f(-1 * incoming_wi_object, w_out); // need to use current isect

        L_out += (ratio * emission * cos_theta_out) / pdf;
      }
    }
  }




  return L_out / (double) total_samples;

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
    
  double cpdf = 0.7;

  // russian roulette termination probability = 1 - continuation probability.
  if (r.depth <= 0) {
    return L_out;
  }

  // Calculate one bounce radiance
  L_out = one_bounce_radiance(r, isect);
  if (r.depth <= 1 || random_uniform() < 1 - cpdf) {
    return L_out;
  }

  // Sample direction;
  Vector3D incoming_wi_object;
  double pdf;
  isect.bsdf->sample_f(w_out, &incoming_wi_object, &pdf);

  Vector3D incoming_wi_world = o2w * incoming_wi_object;

  // Calculate new intersection point
  Ray new_ray = Ray(hit_p, incoming_wi_world);
  new_ray.depth = r.depth - 1;
  new_ray.min_t = EPS_F;

  Intersection new_isect;

  if (bvh->intersect(new_ray, &new_isect)) {
    double cos_theta_out = dot(incoming_wi_object.unit(), Vector3D(0,0,1));
    Vector3D ratio = isect.bsdf->f(-1 * incoming_wi_object, w_out); // need to use current isect

    L_out += ((at_least_one_bounce_radiance(new_ray, new_isect) * cos_theta_out * ratio) / pdf) / cpdf;
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
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;


//  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
//  L_out += at_least_one_bounce_radiance(r, isect) - one_bounce_radiance(r, isect);

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

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  Vector3D total_radiance = Vector3D();
    
  float s1 = 0.0;
  float s2 = 0.0;
  int sample = 1;
    
  for (sample = 1; sample <= num_samples; sample++) {
      // adaptive sampling
      
      Vector2D sample_position = origin + gridSampler->get_sample(); // should return something between ([x, x+1], [y, y+1])
      double normalized_x = sample_position.x / (double) sampleBuffer.w; // normalize coordinates
      double normalized_y = sample_position.y / (double) sampleBuffer.h;

      Ray to_trace = camera->generate_ray(normalized_x, normalized_y);
      to_trace.depth = max_ray_depth;
      Vector3D sample_radiance = est_radiance_global_illumination(to_trace);
      
      float illum = sample_radiance.illum();
      s1 += illum;
      s2 += illum * illum;
      
      // uncomment 1
      total_radiance += sample_radiance;
      //total_radiance += est_radiance_global_illumination(to_trace);
      
      // uncomment 2
      // check only every samplesPerBatch
      if (sample > 1 && sample % samplesPerBatch == 0) {
          float std = std::sqrt(1.0 / (sample - 1) * (s2 - s1*s1 / sample));
          float confidence_interval = 1.96 * std / std::sqrt(sample);
          if (confidence_interval <= maxTolerance * s1 / sample) {
              break;
          }
      }

  }

  
  // uncomment 3
  //total_radiance /= (double) num_samples;
  total_radiance /= (double) sample;

  /*
   * Start of Lens Flare Starburst Experiment:
   */
  Vector3D starburst_radiance = raytrace_starburst(x, y);
//  cout << starburst_radiance << endl;

  sampleBuffer.update_pixel(total_radiance + starburst_radiance, x, y);
    
  //uncomment 4
  //sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
  sampleCountBuffer[x + y * sampleBuffer.w] = sample;

}

Vector3D PathTracer::raytrace_starburst(size_t x, size_t y) {
  Vector3D total_starburst_radiance = Vector3D();
  Vector2D origin = Vector2D(x, y);
  int sample;
  int num_samples = 16;

  for (sample = 1; sample <= num_samples; sample++) {
    Vector2D sample_position = origin + gridSampler->get_sample(); // should return something between ([x, x+1], [y, y+1])
    double normalized_x = sample_position.x / (double) sampleBuffer.w; // normalize coordinates
    double normalized_y = sample_position.y / (double) sampleBuffer.h;

    Vector2D normalized_coord = Vector2D(normalized_x, normalized_y);

    for (int l = 0; l < flare_origins.size(); l++) {
      Vector2D& fo = flare_origins[l];
      Vector2D fo_s = Vector2D(fo.x * (double)sampleBuffer.w, fo.y * (double) sampleBuffer.h);
      double r = max(1.0, (fo_s - sample_position).norm());
      double r2 = r*r;

      total_starburst_radiance += flare_radiance[l] / r2;
    }
  }

  return total_starburst_radiance / (double) num_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
