#ifndef CGL_STATICSCENE_LIGHT_H
#define CGL_STATICSCENE_LIGHT_H

#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"
#include "pathtracer/sampler.h" // UniformHemisphereSampler3D, UniformGridSampler2D
#include "util/image.h"   // HDRImageBuffer

#include "scene.h"  // SceneLight
#include "object.h" // Mesh, SphereObject

namespace CGL { namespace SceneObjects {

// Directional Light //

class DirectionalLight : public SceneLight {
 public:
  DirectionalLight(const Vector3D rad, const Vector3D posLight, const Vector3D lightDir);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return true; }

  Vector3D posLight;
  Vector3D radiance;
private:
  Vector3D dirToLight;

}; // class Directional Light

// Infinite Hemisphere Light //

class InfiniteHemisphereLight : public SceneLight {
 public:
  InfiniteHemisphereLight(const Vector3D rad);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  Vector3D radiance;
  Matrix3x3 sampleToWorld;
  UniformHemisphereSampler3D sampler;

}; // class InfiniteHemisphereLight


// Point Light //

class PointLight : public SceneLight {
 public: 
  PointLight(const Vector3D rad, const Vector3D pos);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return true; }

  Vector3D radiance;
  Vector3D position;
  
}; // class PointLight

// Spot Light //

class SpotLight : public SceneLight {
 public:
  SpotLight(const Vector3D rad, const Vector3D pos, 
            const Vector3D dir, double angle);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return true; }

  Vector3D radiance;
  Vector3D position;
  Vector3D direction;
  double angle;

}; // class SpotLight

// Area Light //

class AreaLight : public SceneLight {
 public:
  AreaLight(const Vector3D rad, 
            const Vector3D pos,   const Vector3D dir, 
            const Vector3D dim_x, const Vector3D dim_y);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  Vector3D radiance;
  Vector3D position;
  Vector3D direction;
  Vector3D dim_x;
  Vector3D dim_y;
  UniformGridSampler2D sampler;
  double area;

}; // class AreaLight

// Sphere Light //

class SphereLight : public SceneLight {
 public:
  SphereLight(const Vector3D rad, const SphereObject* sphere);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  const SphereObject* sphere;
  Vector3D radiance;
  UniformHemisphereSampler3D sampler;

}; // class SphereLight

// Mesh Light

class MeshLight : public SceneLight {
 public:
  MeshLight(const Vector3D rad, const Mesh* mesh);
  Vector3D sample_L(const Vector3D p, Vector3D* wi, double* distToLight,
                    double* pdf) const;
  bool is_delta_light() const { return false; }

  const Mesh* mesh;
  Vector3D radiance;

}; // class MeshLight

} // namespace SceneObjects
} // namespace CGL

#endif  // CGL_STATICSCENE_BSDF_H
