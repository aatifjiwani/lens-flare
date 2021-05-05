#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "util/random_util.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

float CameraApertureTexture::pdf() {
  float total_area = (float) width * height;
  float bounding_box_area = ((float) (max_x - min_x)) * ((float) (max_y - min_y));

  return bounding_box_area / total_area;
}

float CameraApertureTexture::sample_aperture(float &u, float &v) {
  /*
   * Sample (u,v) coordinate in the Aperture's Bounding Box
   * Return the sampled value
   * Set &u, &v to the sampled coordinates
   *
   * !! IMPORTANT: Ensure that the interval for (u) is [-0.5, 0.5] and likewise for (v)
   * !! with (0,0) at the CENTER of the aperture
   */

  // Sample uniform in [0,1]
  double u_sample = random_uniform();
  double v_sample = random_uniform();

  double u_coordinate = min_x + u_sample * (max_x - min_x);
  double v_coordinate = min_y + v_sample * (max_y - min_y);

  int u_pixel = (int) round(u_coordinate); // Pixel Values i.e. 250
  int v_pixel = (int) round(v_coordinate);

  float sampled_value = aperture[v_pixel * width + u_pixel];

  float u_global_coord = (float) u_pixel / (float) width;
  float v_global_coord = (float) v_pixel / (float) height;

  u = u_global_coord - 0.5;
  v = v_global_coord - 0.5;

  return sampled_value;
}

/**
 * Sets the field of view to match screen screenW/H.
 * NOTE: data and screenW/H will almost certainly disagree about the aspect
 *       ratio. screenW/H are treated as the source of truth, and the field
 *       of view is expanded along whichever dimension is too narrow.
 * NOTE2: info.hFov and info.vFov are expected to be in DEGREES.
 */
void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  nClip = info.nClip;
  fClip = info.fClip;
  hFov = info.hFov;
  vFov = info.vFov;

  double ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);
  ar = static_cast<double>(screenW) / screenH;
  if (ar1 < ar) {
    // hFov is too small
    hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
  } else if (ar1 > ar) {
    // vFov is too small
    vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
  }
  screenDist = ((double) screenH) / (2.0 * tan(radians(vFov) / 2));
}

/**
 * This function places the camera at the target position and sets the arguments.
 * Phi and theta are in RADIANS.
 */
void Camera::place(const Vector3D targetPos, const double phi,
                   const double theta, const double r, const double minR,
                   const double maxR) {
  double r_ = min(max(r, minR), maxR);
  double phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
  this->targetPos = targetPos;
  this->phi = phi_;
  this->theta = theta;
  this->r = r_;
  this->minR = minR;
  this->maxR = maxR;
  compute_position();
}

/**
 * This function copies the camera placement state.
 */
void Camera::copy_placement(const Camera& other) {
  pos = other.pos;
  targetPos = other.targetPos;
  phi = other.phi;
  theta = other.theta;
  minR = other.minR;
  maxR = other.maxR;
  c2w = other.c2w;
}

/**
 * This sets the screen size & compute the new FOV.
 */
void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  ar = 1.0 * screenW / screenH;
  hFov = 2 * degrees(atan(((double) screenW) / (2 * screenDist)));
  vFov = 2 * degrees(atan(((double) screenH) / (2 * screenDist)));
}

/**
 * This function translates the camera position
 */
void Camera::move_by(const double dx, const double dy, const double d) {
  const double scaleFactor = d / screenDist;
  const Vector3D displacement =
    c2w[0] * (dx * scaleFactor) + c2w[1] * (dy * scaleFactor);
  pos += displacement;
  targetPos += displacement;
}

/**
 * This function translates the camera position (in forward direction)
 */
void Camera::move_forward(const double dist) {
  double newR = min(max(r - dist, minR), maxR);
  pos = targetPos + ((pos - targetPos) * (newR / r));
  r = newR;
}

/**
 * This function rotates the camera position
 */
void Camera::rotate_by(const double dPhi, const double dTheta) {
  phi = clamp(phi + dPhi, 0.0, (double) PI);
  theta += dTheta;
  compute_position();
}

/**
 * This function computes the camera position, basis vectors, and the view matrix
 */
void Camera::compute_position() {
  double sinPhi = sin(phi);
  if (sinPhi == 0) {
    phi += EPS_F;
    sinPhi = sin(phi);
  }
  const Vector3D dirToCamera(r * sinPhi * sin(theta),
                             r * cos(phi),
                             r * sinPhi * cos(theta));
  pos = targetPos + dirToCamera;
  Vector3D upVec(0, sinPhi > 0 ? 1 : -1, 0);
  Vector3D screenXDir = cross(upVec, dirToCamera);
  screenXDir.normalize();
  Vector3D screenYDir = cross(dirToCamera, screenXDir);
  screenYDir.normalize();

  c2w[0] = screenXDir;
  c2w[1] = screenYDir;
  c2w[2] = dirToCamera.unit();   // camera's view direction is the
                                 // opposite of of dirToCamera, so
                                 // directly using dirToCamera as
                                 // column 2 of the matrix takes [0 0 -1]
                                 // to the world space view direction
}

/**
 * This function stores the camera settings into a file
 */
void Camera::dump_settings(string filename) {
  ofstream file(filename);
  file << hFov << " " << vFov << " " << ar << " " << nClip << " " << fClip << endl;
  for (int i = 0; i < 3; ++i)
    file << pos[i] << " ";
  for (int i = 0; i < 3; ++i)
    file << targetPos[i] << " ";
  file << endl;
  file << phi << " " << theta << " " << r << " " << minR << " " << maxR << endl;
  for (int i = 0; i < 9; ++i)
    file << c2w(i/3, i%3) << " ";
  file << endl;
  file << screenW << " " << screenH << " " << screenDist << endl;
  file << focalDistance << " " << lensRadius << endl;
  cout << "[Camera] Dumped settings to " << filename << endl;
}

/**
 * This function loads the camera settings from a file
 */
void Camera::load_settings(string filename) {
  ifstream file(filename);

  file >> hFov >> vFov >> ar >> nClip >> fClip;
  for (int i = 0; i < 3; ++i)
    file >> pos[i];
  for (int i = 0; i < 3; ++i)
    file >> targetPos[i];
  file >> phi >> theta >> r >> minR >> maxR;
  for (int i = 0; i < 9; ++i)
    file >> c2w(i/3, i%3);
  file >> screenW >> screenH >> screenDist;
  file >> focalDistance >> lensRadius;
  cout << "[Camera] Loaded settings from " << filename << endl;
}

// world coordinates to screen coordinates
void Camera::analyze_world_coord(Vector3D& pos_world, double& ns_x, double& ns_y) {
  double hFOV_rads = this->hFov * (PI / 180.0); // x
  double vFOV_rads = this->vFov * (PI / 180.0); // y

  double edge_x = tan(0.5 * hFOV_rads); //
  double edge_y = tan(0.5 * vFOV_rads);

  Vector3D pos_camera = c2w.T() * (pos_world - pos);
  cout << "position of coord in camera: " << pos_camera << endl;

  Vector3D pos_image = pos_camera / abs(pos_camera.z);
  cout << "position of coord on image plane in camera: " << pos_image << endl;

  cout << "camera image plane edges x,y: " << edge_x << ", " << edge_y << endl;

//  double camera_space_x = edge_x * (2 * x - 1);
//  double camera_space_y = edge_y * (2 * y - 1);
  /*
   * (csx / ex) = 2x - 1
   * (csx/ex) + 1 = 2x
   * x = ((csx/ex)+1)/2
   */

  // Normalized Screen Coords
  ns_x = ((pos_image.x / edge_x)+1)/2.0;
  ns_y = ((pos_image.y / edge_y)+1)/2.0;

  cout << "normalized screen space coords: (x,y) = (" << ns_x << ", " << ns_y << ")\n";
}

/**
 * This function generates a ray from camera perspective, passing through camera / sensor plane (x,y)
 */
Ray Camera::generate_ray(double x, double y) const {

  // TODO (Part 1.1):
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  //

  double hFOV_rads = this->hFov * (PI / 180.0); // x
  double vFOV_rads = this->vFov * (PI / 180.0); // y

  double edge_x = tan(0.5 * hFOV_rads); //
  double edge_y = tan(0.5 * vFOV_rads);

  double camera_space_x = edge_x * (2 * x - 1);
  double camera_space_y = edge_y * (2 * y - 1);
  Vector3D direction = Vector3D(camera_space_x, camera_space_y, -1).unit();

  Vector3D direction_world = c2w * direction;

  // Pos is already the camera position in world space
  Ray to_generate = Ray(pos, direction_world);
  to_generate.min_t = nClip;
  to_generate.max_t = fClip;

  return to_generate;

}

} // namespace CGL
