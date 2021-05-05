#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"

#include <cstdlib>
#include <complex>

using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
	
	//initialize ghost_buffer
	ghost_buffer = HDRImageBuffer();
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
				
				// TODO: should this be a list?
				// instantiate: angle_to_sun
				Matrix3x3 w2c = camera->c2w.inv();
				Vector3D cam_dirToLight = w2c*dlight->dirToLight; // towards sun
				cam_dirToLight.normalize();
				float angle_to_sun = dot(cam_dirToLight, Vector3D(0, 0, 1)); // todo: z points out and positive?
			  Vector2D axis_ray = Vector2D(ns_x, ns_y);
				
				//flare_angles.push_back(angle_to_sun);
				//flare_axis_rays.push_back(axis_ray);
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

  // Sample environment light
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  // TODO (Part 3): Return the direct illumination.
  L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
//  L_out += at_least_one_bounce_radiance(r, isect) - one_bounce_radiance(r, isect);

  return L_out;
}

// draw one ghost to ghost_buffer
void PathTracer::draw_ghost(string color, float r1, float r2) {
	
//	float sun_angle = flare_angles.back(); // change to support multiple suns
//	Vector2D axis_ray = flare_axis_rays.back();
	
//	float shift_amt = -(r1+r2)/2;
//	float scale_amt = abs(r2-r1);
//
//	//define 4 points
//	Vector3D p1 = Vector3D(-1, -1, 1);
	
	
	// run shift_vertex on points
	
	
	// rasturize 2 textured triangles to ghost_buffer
	//ghost_aperture_function->sample_aperture(u, v);
	//camera->ghost_texture?
	// todo: proj 1 code
	
	
	
	
	
	//tests
	
	// test 1: draw line on axis
	// test: make sure it's additive
	// TODO: get actual color from aperature texture.
	// test: one ghost w/ weird aperature
	
	// test: figure out scaling
	Vector3D test = Vector3D(1, 1, 1);
	

	for(int i = int(ghost_buffer.w/2); i<ghost_buffer.w; i++) {
		for(int j = int(ghost_buffer.h/2); j< ghost_buffer.h; j++) {
			ghost_buffer.update_pixel_additive(test, i, j);
		}
	}
	
	
	
	
}

void PathTracer::generate_ghost_buffer() {
	
	// initialize size
	
	ghost_buffer.resize(sampleBuffer.w, sampleBuffer.h);
	// get sun angle and axis ray
	
	

	
	// run every ghost func on each wavelength
	// additively store stuff in ghost buffer
	
	//test
	draw_ghost("red", -5, 5);
	
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

  // BEGIN UNCOMMENT
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
// END UNCOMMENT

  /*
   * Start of Lens Flare Starburst Experiment:
   */
  // Vector3D starburst_radiance = raytrace_starburst(x, y); // TODO: add back
	//cout << ghost_buffer << "ghost_buffer";
	Vector3D ghost_color = ghost_buffer.get_pixel_value(x, y); // TODO: representing at Vec3D for now...
//  cout << "(x, y, radiance): (" << x << ", " << y << ", " << starburst_radiance << ")\n";
//  Vector3D starburst_radiance = raytrace_starburst_experiment(x, y);
//  cout << starburst_radiance << endl;
//  cout << "total radiance: " << total_radiance << ", starburst: " << starburst_radiance << endl;
  sampleBuffer.update_pixel(total_radiance + ghost_color, x, y);
    
  //uncomment 4
  //sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
  sampleCountBuffer[x + y * sampleBuffer.w] = sample;

}

std::complex<double> complex_exp(double exponent, bool negative) {
  /*
   * Perform e^(-1 * j * 2 * PI * exponent)
   * = cos(-2 * PI * exponent) + jsin(-2 * PI * exponent);
   * = cos(2 * PI * exponent) - jsin(2 * PI * exponent);
   */
  double cos_value = cos(2.0 * M_PI * exponent);
  double sin_value = sin(2.0 * M_PI * exponent);
  if (negative)
    sin_value *= -1.0;

  std::complex<double> exponential(cos_value, sin_value);

  return exponential;
}

std::complex<double> PathTracer::compute_phase(int flare, double u, double v, Vector2D& screen_pos) {
  Vector2D& fo = flare_origins[flare];

  // convert [0, 1] x [0, 1] to [0, width] x [0, height]
  auto lr = (double) (ceil(fo.x * (double)sampleBuffer.w));
  auto ud = (double) (ceil(fo.y * (double)sampleBuffer.h));
  screen_pos = Vector2D(lr, ud);

  // convert [0, width] x [0, height] to [-width / 2, width / 2] x [-height / 2, height / 2]
  // smaller heights are higher up
  lr -= (double)sampleBuffer.w / 2.0;
  ud = -ud + (double)sampleBuffer.h / 2.0;

  return complex_exp(u * lr + v * ud, false);
}

double convertCoordinate(size_t pixel_coord, int length, bool y){
  double coord_center;
  if (y){
    coord_center = -((float) pixel_coord) + ((float)length / 2.0);
  } else {
    coord_center = ((float) pixel_coord) - ((float)length / 2.0);
  }

  if (coord_center >= 0) {
    return coord_center;
  }
  return length + coord_center;
}

Vector3D PathTracer::raytrace_starburst(size_t x, size_t y) {
  double xprime, yprime;
  std::complex<double> complex_intensity;
  Vector3D total_starburst_radiance = Vector3D();
  Vector2D curr_screen_pose = Vector2D(x, y);
  Vector2D flare_origin;
  CameraApertureTexture* aperture_function = camera->aperture_texture;
  int num_samples = 300;

  xprime = convertCoordinate(x, sampleBuffer.w, false);
  yprime = convertCoordinate(y, sampleBuffer.h, true);

  for (int yc = aperture_function->min_y; yc <= aperture_function->max_y; yc++) {
    for (int xc = aperture_function->min_x; xc <= aperture_function->max_x; xc++) {
      double sampled_value = (double) aperture_function->aperture[yc*aperture_function->width + xc];
      double u = ((double)xc / (double)aperture_function->width) - 0.5;
      double v = ((double)yc / (double)aperture_function->width) - 0.5;

      double exponent = u * xprime + v * yprime;
      std::complex<double> complex_exponential = complex_exp(exponent, true);

      std::complex<double> additional_phase = compute_phase(0, u, v, flare_origin);

      std::complex<double> intensity_uv = (sampled_value * additional_phase * complex_exponential);

      complex_intensity += intensity_uv;
    }
  }

  double abs_avg_complex_intensity = abs(complex_intensity) / aperture_function->total_value;
  double radius = 30.0;
  // Flare Suppression
  if ((flare_origin - curr_screen_pose).norm() > ((double) aperture_function->width / 2.0)) {
    double norm_coord = (flare_origin - curr_screen_pose).norm();
    double factor = ((double) aperture_function->width / 2.0) / norm_coord; //[0 - 1]
    double factor2 = pow(factor, 8.0);

    abs_avg_complex_intensity = factor2 * abs_avg_complex_intensity;
  }
  else if ((flare_origin - curr_screen_pose).norm() <= radius) {
    // Flare Amplification
    double norm_coord = (flare_origin - curr_screen_pose).norm();
    double factor = norm_coord / radius; //[0 - 1]

    abs_avg_complex_intensity = pow(abs_avg_complex_intensity, factor);
  }

  for (auto & l : flare_radiance) {
    total_starburst_radiance += pow(abs_avg_complex_intensity, 2.0) * l;
  }

  Vector3D radiance_falloff = calculate_irradiance_falloff(x, y, 5.0);

  return total_starburst_radiance + radiance_falloff;

//  for (int s = 0; s < num_samples; s++) {
//    float u, v;
//
//    /*
//     * Calculate:
//     * A[u_i, v_i] * exp(-j * 2 * PI * (u * xprime + v * yprime)) / pdf()
//     */
//
//    float A_mn = aperture_function->sample_aperture(u, v);
//    if (A_mn == 0) {
//      continue;
//    }
//
//    if (x == 250 && y == 250) {
//      cout << A_mn << endl;
//    }
//
//    float exponent = u * xprime + v * yprime;
//    std::complex<float> complex_exponential = complex_exp(exponent);
//
//    float p = aperture_function->pdf();
//
//    std::complex<float> intensity_uv = (A_mn * complex_exponential);
//    float real_part = intensity_uv.real();
//
//    if (x == 250 && y == 250) {
//      cout << "real part " << real_part << endl;
//      cout << intensity_uv << endl;
//    }
//
//    complex_intensity += intensity_uv;
//  }


//  std::complex<float> avg_complex_intensity = complex_intensity / (float)num_samples;
}

Vector3D PathTracer::calculate_irradiance_falloff(size_t x, size_t y, double radius) {
  Vector3D total_starburst_radiance = Vector3D();
  Vector2D origin = Vector2D(x, y);
  int sample;
  int num_samples = 16;

  for (sample = 1; sample <= num_samples; sample++) {
    Vector2D sample_position = origin + gridSampler->get_sample(); // should return something between ([x, x+1], [y, y+1])

    for (int l = 0; l < flare_origins.size(); l++) {
      Vector2D& fo = flare_origins[l];
      Vector2D fo_s = Vector2D(fo.x * (double)sampleBuffer.w, fo.y * (double) sampleBuffer.h);
      double r = 1 + max(0.0, (fo_s - sample_position).norm() - radius);
      double r2 = pow(r, 1.5);

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
