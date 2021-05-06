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
				angle_to_sun = atan(ns_y/ns_x);
				//acos(dot(cam_dirToLight, Vector3D(0, 0, -1))); // todo: z points out and positive?
				cout << ns_x << ns_y << "test";
			  axis_ray = Vector2D(ns_x, ns_y);
				
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


void PathTracer::fill_textured_pixel(float x0, float y0, float u0, float v0, float x1, float y1, float u1, float v1, float x2, float y2, float u2, float v2, int x, int y, Vector3D ghost_color) {
 // assumes correct winding
	// assumes in bounds
	// don't fill if any bary coords are <= 0
	float xy_to_01 = -(y1-y0)*(x-x0) + (x1 - x0)*(y-y0);
	float two_to_01 = -(y1-y0)*(x2-x0) + (x1 - x0)*(y2-y0);

	float alpha = xy_to_01/two_to_01;
//	cout << "test1: " << xy_to_01 << two_to_01 << "end test";
	float xy_to_12 = -(y2-y1)*(x-x1) + (x2 - x1)*(y-y1);
	float zero_to_12 = -(y2-y1)*(x0-x1) + (x2 - x1)*(y0-y1);
	float beta = xy_to_12/zero_to_12;
//	cout << "test1: " << xy_to_12 << zero_to_12 << "end test";

	float gamma = 1 - alpha - beta;
	
	
//	cout << x0 << x1 << x2 << y0 << y1 << y2;
//	cout << xy_to_01 << two_to_01 << alpha << xy_to_12 <<zero_to_12;
//
//	cout<< gamma << alpha << beta << "test";
	if (gamma >= 0 and alpha >= 0 and beta >= 0) {
		float u = u2*alpha + u0*beta + u1*gamma;
		float v = v2*alpha + v0*beta + v1*gamma;
		Vector2D uv = Vector2D(u, v);
		
//		cout << "x: " << x << "y: " << y << "u: " << u << "v: " << v << endl;
	
		
		//TODO: init before?
		CameraApertureTexture* ghost_ap_tex = camera->ghost_aperture_texture;
		std::vector<float>* ghost_aperture_pixels = &ghost_ap_tex->aperture;
		
		float sample = (*ghost_aperture_pixels)[int(floor(uv.y)*ghost_ap_tex->width + uv.x)]; //grayscale
		
		ghost_buffer.update_pixel_additive(sample*ghost_color, x, y);
	}
	
}


void PathTracer::rasterize_textured_triangle(float x0, float y0, float u0, float v0,
	float x1, float y1, float u1, float v1,
	float x2, float y2, float u2, float v2, Vector3D ghost_color)
{
	if(y1<y0) {
		swap(x0, x1);
		swap(y0, y1);
		swap(u0, u1);
		swap(v0, v1);

	}
	if (y2<y0) {
		swap(x0, x2);
		swap(y0, y2);
		swap(u0, u2);
		swap(v0, v2);

	}
	if (y2<y1) {
		swap(x1, x2);
		swap(y1, y2);
		swap(u1, u2);
		swap(v1, v2);
	}
	assert(y0 <= y1 && y1 <= y2);
	
//	int sr = (int)sqrt(sample_rate);

//	x0 *= sr;
//	y0 *= sr;
//	x1 *= sr;
//	y1 *= sr;
//	x2 *= sr;
//	y2 *= sr;
	
	// we can think of our centers of pixels as integer coordinates with this.
	
	x0-=0.5;
	y0-=0.5;
	x1-=0.5;
	y1-=0.5;
	x2-=0.5;
	y2-=0.5;
	
	//find bounding box
	float min_x = max(0, int(floor(min(min(x0, x1), x2))));
	float max_x = min(int(ghost_buffer.w - 1), int(ceil(max(max(x0, x1), x2))));
	float min_y = max(0, int(floor(y0)));
	float max_y = min(int(ghost_buffer.h - 1), int(ceil(y2)));
//
//	bool sample_nearest = false;
//	if(psm == P_NEAREST) {
//		sample_nearest = true;
//	}
	

	for (int y = min_y; y < max_y; y++) {
		for (int x = min_x; x < max_x; x++) {
			fill_textured_pixel(x0, y0, u0, v0, x1, y1, u1, v1, x2, y2, u2, v2, x, y, ghost_color);
		}
	}



}

Vector2D PathTracer::shift_vertex(float x, float y, float scale, float shift_amount) {
	Vector3D v = Vector3D(x, y, 1);
	float new_angle_to_sun = angle_to_sun;
	//atan(axis_ray.y / axis_ray.x);
	Matrix3x3 scaling = Matrix3x3(scale, 0, 0,
																0, scale, 0,
																0, 0, 1);
	Matrix3x3 rotation = Matrix3x3(cos(new_angle_to_sun), -sin(new_angle_to_sun), 0,
																 sin(new_angle_to_sun), cos(new_angle_to_sun), 0,
																 0, 0, 1);
	
	Matrix3x3 shift = Matrix3x3(1, 0, shift_amount*cos(new_angle_to_sun),
															0, 1, shift_amount*sin(new_angle_to_sun),
															0, 0, 1);
	Vector3D result = shift*rotation*scaling*v;
																	
	return Vector2D(result.x, result.y);
																	 
																	 }

// draw one ghost to ghost_buffer
void PathTracer::draw_ghost(string color, float r1, float r2) {
	
	float i = ghost_buffer.w/2;
	float j = ghost_buffer.h/2;
	
	cout << axis_ray << "axis_ray" << endl;
	cout << angle_to_sun << "angle" << endl;
	cout << atan(axis_ray.y / axis_ray.x) << "atan" << endl;
	cout << acos(angle_to_sun) << "acos" << endl;
	
//	while(i<ghost_buffer.w and j < ghost_buffer.h){
//		ghost_buffer.update_pixel_additive(test, int(i), int(j));
//		i+= axis_ray[0];
//		j+= axis_ray[1];
//	}
	// end test
	
	
	// rasturize 2 textured triangles to ghost_buffer
	
	// given r1, r2, and color, draw ghost
	
	float shift_amt = -(r1+r2)/2 * 0.4;
	float scale_amt = abs(r2-r1) * 0.05;
	
//	shift_amt = -50;
//	scale_amt = 20;
	
//	float gb_mid_w = flare_origins[0].x / 2.0; ghost_buffer.w/2;
//	float gb_mid_h = flare_origins[0].y / 2;
  double gb_mid_w = (double) (ceil(axis_ray.x * (double)ghost_buffer.w));
  double gb_mid_h = (double) (ceil(axis_ray.y * (double)ghost_buffer.h));
	
	//define 4 points in screenspace
//	Vector2D ul = shift_vertex(gb_mid_w-100, gb_mid_h+100, scale_amt, shift_amt);
//	Vector2D ll = shift_vertex(gb_mid_w-100, gb_mid_h-100, scale_amt, shift_amt);
//	Vector2D ur = shift_vertex(gb_mid_w+100, gb_mid_h+100, scale_amt, shift_amt);
//	Vector2D lr = shift_vertex(gb_mid_w+100, gb_mid_h-100, scale_amt, shift_amt);
	
  Vector2D ul = shift_vertex(-1, 1, scale_amt, -shift_amt);
  Vector2D ll = shift_vertex(-1, -1, scale_amt, -shift_amt);
  Vector2D ur = shift_vertex(1, 1, scale_amt, -shift_amt);
  Vector2D lr = shift_vertex(1, -1, scale_amt, -shift_amt);
	
	// TODO: add sun point instead of midpoint
	
	Vector3D ghost_color = Vector3D();
	if (color == "red") {
		ghost_color = Vector3D(1, 0, 0);
	} else if (color == "green") {
		ghost_color = Vector3D(0, 1, 0);
	} else {
		ghost_color = Vector3D(0, 0, 1);
	}
	
	float intensity_scalar = 10;
	float size_scalar = 1/(scale_amt*scale_amt);
	ghost_color *= intensity_scalar * size_scalar;
	
	rasterize_textured_triangle(gb_mid_w+ul.x, gb_mid_h+ul.y, 0, 0, gb_mid_w+ll.x, gb_mid_h+ll.y, 0, camera->ghost_aperture_texture->height, gb_mid_w+ur.x, gb_mid_h+ur.y, camera->ghost_aperture_texture->width, 0, ghost_color);
	
	rasterize_textured_triangle(gb_mid_w+lr.x, gb_mid_h+lr.y, 0, 0, gb_mid_w+ll.x, gb_mid_h+ll.y, 0, camera->ghost_aperture_texture->height, gb_mid_w+ur.x, gb_mid_h+ur.y, camera->ghost_aperture_texture->width, 0, ghost_color);
	
//	rasterize_textured_triangle(gb_mid_w+100, gb_mid_h-100, camera->ghost_aperture_texture->width, camera->ghost_aperture_texture->height, gb_mid_w-100, gb_mid_h-100, 0, camera->ghost_aperture_texture->height, gb_mid_w+100, gb_mid_h+100, camera->ghost_aperture_texture->width, 0);
	
	
	// aperature at center
//	rasterize_textured_triangle(gb_mid_w-100, gb_mid_h+100, 0, 0, gb_mid_w-100, gb_mid_h-100, 0, camera->ghost_aperture_texture->height, gb_mid_w+100, gb_mid_h+100, camera->ghost_aperture_texture->width, 0);
//
//	rasterize_textured_triangle(gb_mid_w+100, gb_mid_h-100, camera->ghost_aperture_texture->width, camera->ghost_aperture_texture->height, gb_mid_w-100, gb_mid_h-100, 0, camera->ghost_aperture_texture->height, gb_mid_w+100, gb_mid_h+100, camera->ghost_aperture_texture->width, 0);
	
}

// begin integration
Matrix3x3 make_2_matrix(float a, float b, float c, float d) {
		// | a b 0 |
		// | c d 0 |
		// | 0 0 0 |
		return Matrix3x3(a, b, 0, c, d, 0, 0, 0, 0);
}

// a 2x2 inversion on a zero-padded 3x3 matrix
Matrix3x3 invert2x2(Matrix3x3 mat) {
		float a = mat(0, 0);
		float b = mat(0, 1);
		float c = mat(1, 0);
		float d = mat(1, 1);
		return 1.0 / (a*d - b*c) * make_2_matrix(d, -b, -c, a);
}

Matrix3x3 T(float d) {
		return make_2_matrix(1, d, 0, 1);
}

Matrix3x3 generate_R(float c, float n1, float n2) {
		return make_2_matrix(1, 0, c*(n1-n2)/n2, n1/n2);
}

Matrix3x3 generate_L(float c) {
		return make_2_matrix(1, 0, 2*c, 1);
}

float pupil_height = 12.3;

Matrix3x3 Ts[] = {
		T(7.700),  // 1
		T(1.850),
		T(3.520),
		T(1.850),
		T(4.180),
		T(3.000),
		T(1.850),
		T(7.270),
		T(83.91)
};

float red_refr[] = {1.652, 1.5991, 1, 1.6396, 1, 1, 1.5776, 1.68990, 1};
float green_refr[] = {1.652, 1.6113, 1, 1.65, 1, 1, 1.5885, 1.6999, 1};
float blue_refr[] = {1.652, 1.6164, 1, 1.6542, 1, 1, 1.5930, 1.7040, 1};
float curvatures[] = {1/30.810, 1/-89.350, 1/580.380, 1/-80.630, 1/28.340, 0, 0, 1/32.190, 1/-52.990, 1/81.320};

// assumes color_
std::vector<Matrix3x3> create_Rs_for_color(float color_refr[]) {
		std::vector<Matrix3x3> arr;
		float prev_n = 1.00;
		// NOTE: hard coded array length for 9 lenses!
		for (int i = 0; i < 9; i++) {
				arr.push_back(generate_R(curvatures[i], prev_n, color_refr[i]));
				prev_n = color_refr[i];
		}
		return arr;
}

std::vector<Matrix3x3> create_Ls() {
		std::vector<Matrix3x3> arr;
		for (int i = 0; i < 9; i++) {
				arr.push_back(generate_L(curvatures[i]));
		}
		return arr;
}

float radii_size(float r1, float r2) {
		return (r1 - r2) * (r1 - r2);
}

std::vector<Matrix3x3> Ls = create_Ls();

std::vector<Matrix3x3> R_red = create_Rs_for_color(red_refr);
std::vector<Matrix3x3> R_blue = create_Rs_for_color(blue_refr);
std::vector<Matrix3x3> R_green = create_Rs_for_color(green_refr);

Vector2D trace_ray_auto_before(float r, float theta, int i, int j, std::vector<Matrix3x3> color_R) {
		// mapping to 3D
		Vector3D ray = Vector3D(r, theta, 0);
		
		int mini = std::min(i, j);
		int maxj = std::max(i, j);
		i = mini;
		j = maxj;
		
		Matrix3x3 M = make_2_matrix(1, 0, 0, 1);
		
		// forward through j-1
		for (int k = 0; k < j; k++)
				M = Ts[k] * color_R[k] * M;
		
		// reflect off of Lj
		M = Ls[j] * M;
		
		// cout << "-1. no inv alr" << endl << M << endl;
		
		for (int k = j - 1; k > i; k--)
				M = invert2x2(color_R[k]) * Ts[k] * M;
		
		// cout << "0. inv alr" << endl << M << endl;
		
		M = Ts[i] * invert2x2(Ls[i]) * Ts[i] * M;
		
		// cout << "1." << endl << M << endl;
		
		// zero indexed
		for (int k = i+1; k < 9; k++) {
				if (k == 5) {
						Vector3D after_ap = M * ray;
						if (after_ap.x > 11.6 || after_ap.x < -11.6) {
								// cout << "recasting, got aperature status: " << after_ap.x << endl;
								float r_a = 11.6;
								if (r < 0)
										r_a = -11.5;
								float r_e = (r_a - M(0, 1) * theta) / M(0, 0);
								ray = Vector3D(r_e, theta, 0);
								// cout << "ray we're casting: " << ray << endl;
						}
						// crossing the aperture
						M = Ts[k] * M;
						continue;
				}
				M = Ts[k] * color_R[k] * M;
		}
		
		// cout << "2." << endl << M << endl;
		
		Vector3D res = M * ray;
		return Vector2D(res.x, res.y);
}

Vector2D trace_ray_auto_after(float r, float theta, int i, int j, std::vector<Matrix3x3> color_R) {
    Vector3D ray = Vector3D(r, theta, 0);
    
    int mini = std::min(i, j);
    int maxj = std::max(i, j);
    i = mini;
    j = maxj;
    
    Matrix3x3 M = make_2_matrix(1, 0, 0, 1);
    
    for (int k = 0; k < j; k++) {
        if (k == 5) {
            Vector3D after_ap = M * ray;
            if (after_ap.x > 11.6 || after_ap.x < -11.6) {
                // cout << "recasting, got aperature status: " << after_ap.x << endl;
                float r_a = 11.6;
                if (r < 0)
                    r_a = -11.5;
                float r_e = (r_a - M(0, 1) * theta) / M(0, 0);
                ray = Vector3D(r_e, theta, 0);
                // cout << "ray we're casting: " << ray << endl;
            }
            // crossing the aperture
            M = Ts[k] * M;
            continue;
        }
        M = Ts[k] * color_R[k] * M;
    }
    
    // reflect off of Lj
    M = Ls[j] * M;
    
    for (int k = j-1; k > i; k--)
        M = invert2x2(color_R[k]) * Ts[k] * M;
    
    // cout << "0. inv alr" << endl << M << endl;
    
    M = Ts[i] * invert2x2(Ls[i]) * Ts[i] * M;
    
    // forward through to the end
    for (int k = i+1; k < 9; k++)
        M = Ts[k] * color_R[k] * M;
    
    Vector3D res = M * ray;
    return Vector2D(res.x, res.y);
    
}


// end integration

void PathTracer::generate_ghost_buffer() {
	
	// initialize size
	
	cout << "sampleBuffer.w" << sampleBuffer.w << "sampleBuffer.h: "<< sampleBuffer.h;
	ghost_buffer.resize(sampleBuffer.w, sampleBuffer.h);
	// get sun angle and axis ray
	
	// if there's no sun, don't do this function
	if (axis_ray.x == 0 and axis_ray.y == 0) {
		return;
	}

	// run every ghost func on each wavelength
	// additively store stuff in ghost buffer
	
	//test
	
	for (int i = 0; i < 5; i++) {
			for (int j = i+1; j < 5; j++) {
					Vector2D sensor_ray_1 = trace_ray_auto_before(14.5, angle_to_sun, i, j, R_red);
					Vector2D sensor_ray_2 = trace_ray_auto_before(-14.5, angle_to_sun, i, j, R_red);
					draw_ghost("red", sensor_ray_1.x, sensor_ray_2.x);
					sensor_ray_1 = trace_ray_auto_before(14.5, angle_to_sun, i, j, R_green);
					sensor_ray_2 = trace_ray_auto_before(-14.5, angle_to_sun, i, j, R_green);
					draw_ghost("green", sensor_ray_1.x, sensor_ray_2.x);
					sensor_ray_1 = trace_ray_auto_before(14.5, angle_to_sun, i, j, R_blue);
					sensor_ray_2 = trace_ray_auto_before(-14.5, angle_to_sun, i, j, R_blue);
					draw_ghost("blue", sensor_ray_1.x, sensor_ray_2.x);
			}
	}
    
    for (int i = 6; i < 9; i++) {
        for (int j = i+1; j < 9; j++) {
					Vector2D sensor_ray_1 = trace_ray_auto_after(14.5, angle_to_sun, i, j, R_red);
					Vector2D sensor_ray_2 = trace_ray_auto_after(-14.5, angle_to_sun, i, j, R_red);
					draw_ghost("red", sensor_ray_1.x, sensor_ray_2.x);
					sensor_ray_1 = trace_ray_auto_after(14.5, angle_to_sun, i, j, R_green);
					sensor_ray_2 = trace_ray_auto_after(-14.5, angle_to_sun, i, j, R_green);
					draw_ghost("green", sensor_ray_1.x, sensor_ray_2.x);
					sensor_ray_1 = trace_ray_auto_after(14.5, angle_to_sun, i, j, R_blue);
					sensor_ray_2 = trace_ray_auto_after(-14.5, angle_to_sun, i, j, R_blue);
					draw_ghost("blue", sensor_ray_1.x, sensor_ray_2.x);
        }
    }
	
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
  /*
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
   */
// END UNCOMMENT

  /*
   * Start of Lens Flare Starburst Experiment:
   */
  //Vector3D starburst_radiance = raytrace_starburst(x, y); // TODO: add back

	//cout << ghost_buffer << "ghost_buffer";
	Vector3D ghost_color = ghost_buffer.get_pixel_value(x, y); // TODO: representing at Vec3D for now...
//  cout << "(x, y, radiance): (" << x << ", " << y << ", " << starburst_radiance << ")\n";
//  Vector3D starburst_radiance = raytrace_starburst_experiment(x, y);
//  cout << starburst_radiance << endl;
//  cout << "total radiance: " << total_radiance << ", starburst: " << starburst_radiance << endl;


    sampleBuffer.update_pixel(total_radiance + ghost_color, x, y);



  //uncomment 4
  //sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
  //sampleCountBuffer[x + y * sampleBuffer.w] = sample;

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
  double radius = flare_radius;
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

  double intensity = -flare_intensity + 3.0;
  if (intensity <= 0) {
    intensity = 2.0;
  }
  for (auto & l : flare_radiance) {
    total_starburst_radiance += pow(abs_avg_complex_intensity, intensity) * l;
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
