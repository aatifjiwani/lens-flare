#ifndef CGL_PATHTRACER_H
#define CGL_PATHTRACER_H

#include <complex>

#include "CGL/timer.h"

#include "scene/bvh.h"
#include "pathtracer/sampler.h"
#include "pathtracer/intersection.h"

#include "application/renderer.h"

#include "scene/scene.h"
using CGL::SceneObjects::Scene;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHNode;
using CGL::SceneObjects::BVHAccel;

namespace CGL {

    class PathTracer {
    public:
        PathTracer();
        ~PathTracer();

        /**
         * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
         * RENDERING, or DONE), transitions to READY b/c a changing window size
         * would invalidate the output. If in INIT and configuration is done,
         * transitions to READY.
         * \param width width of the frame
         * \param height height of the frame
         */
        void set_frame_size(size_t width, size_t height);

        void write_to_framebuffer(ImageBuffer& framebuffer, size_t x0, size_t y0, size_t x1, size_t y1);

        /**
         * Testing functions
         */
         void find_sun_pos();

        /**
         * If the pathtracer is in READY, delete all internal data, transition to INIT.
         */
        void clear();

        void autofocus(Vector2D loc);

        /**
         * Trace an ray in the scene.
         */
        Vector3D estimate_direct_lighting_hemisphere(const Ray& r, const SceneObjects::Intersection& isect);
        Vector3D estimate_direct_lighting_importance(const Ray& r, const SceneObjects::Intersection& isect);

        Vector3D est_radiance_global_illumination(const Ray& r);
        Vector3D zero_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect);
        Vector3D one_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect);
        Vector3D at_least_one_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect);
        
        Vector3D debug_shading(const Vector3D d) {
            return Vector3D(abs(d.r), abs(d.g), .0).unit();
        }

        Vector3D normal_shading(const Vector3D n) {
            return n * .5 + Vector3D(.5);
        }

        /**
         * Trace a camera ray given by the pixel coordinate.
         */
        void raytrace_pixel(size_t x, size_t y);

        /**
         * Trace the starburst given by the pixel coordinate.
         */
        double flare_radius;
        double flare_intensity;
        Vector3D calculate_irradiance_falloff(size_t x, size_t y, double radiance);
        Vector3D raytrace_starburst(size_t x, size_t y);

        /**
         * Compute the phase value given the horizontal and vertical shifts for the flare at the given index.
         */
        std::complex<double> compute_phase(int flare, double u, double v, Vector2D& screen_pos);

        // Integrator sampling settings //

        size_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
        size_t ns_aa;         ///< number of camera rays in one pixel (along one axis)
        size_t ns_area_light; ///< number samples per area light source
        size_t ns_diff;       ///< number of samples - diffuse surfaces
        size_t ns_glsy;       ///< number of samples - glossy surfaces
        size_t ns_refr;       ///< number of samples - refractive surfaces

        size_t samplesPerBatch;
        double maxTolerance;
        bool direct_hemisphere_sample; ///< true if sampling uniformly from hemisphere for direct lighting. Otherwise, light sample

        // Components //

        BVHAccel* bvh;                 ///< BVH accelerator aggregate
        EnvironmentLight* envLight;    ///< environment map
        Sampler2D* gridSampler;        ///< samples unit grid
        Sampler3D* hemisphereSampler;  ///< samples unit hemisphere
        HDRImageBuffer sampleBuffer;   ///< sample buffer
        Timer timer;                   ///< performance test timer

        std::vector<int> sampleCountBuffer;   ///< sample count buffer

        Scene* scene;         ///< current scene
        Camera* camera;       ///< current camera

        std::vector<Vector2D> flare_origins; ///< normalized screen space origins of flares
        std::vector<Vector3D> flare_radiance; ///< radiance of captured directional light sources

        // Tonemapping Controls //

        double tm_gamma;                           ///< gamma
        double tm_level;                           ///< exposure level
        double tm_key;                             ///< key value
        double tm_wht;                             ///< white point
    };

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
