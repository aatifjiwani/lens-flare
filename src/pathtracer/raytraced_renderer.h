#ifndef CGL_RAYTRACER_H
#define CGL_RAYTRACER_H

#include <stack>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <algorithm>

#include "CGL/timer.h"

#include "scene/bvh.h"
#include "pathtracer/camera.h"
#include "pathtracer/sampler.h"
#include "util/image.h"
#include "util/work_queue.h"
#include "pathtracer/intersection.h"

#include "application/renderer.h"

#include "scene/scene.h"
using CGL::SceneObjects::Scene;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHNode;
using CGL::SceneObjects::BVHAccel;

#include "pathtracer.h"

namespace CGL {

struct WorkItem {

  // Default constructor.
  WorkItem() : WorkItem(0, 0, 0, 0) { }

  WorkItem(int x, int y, int w, int h)
      : tile_x(x), tile_y(y), tile_w(w), tile_h(h) {}

  int tile_x;
  int tile_y;
  int tile_w;
  int tile_h;

};

/**
 * A pathtracer with BVH accelerator and BVH visualization capabilities.
 * It is always in exactly one of the following states:
 * -> INIT: is missing some data needed to be usable, like a camera or scene.
 * -> READY: fully configured, but not rendering.
 * -> VISUALIZE: visualizatiNG BVH aggregate.
 * -> RENDERING: rendering a scene.
 * -> DONE: completed rendering a scene.
 */
class RaytracedRenderer : public OfflineRenderer {
public:

  /**
   * Default constructor.
   * Creates a new pathtracer instance.
   */
  RaytracedRenderer(size_t ns_aa = 1, 
             size_t max_ray_depth = 4, size_t ns_area_light = 1,
             size_t ns_diff = 1, size_t ns_glsy = 1, size_t ns_refr = 1,
             size_t num_threads = 1,
             size_t samples_per_batch = 32,
             float max_tolerance = 0.05f,
             HDRImageBuffer* envmap = NULL,
             bool direct_hemisphere_sample = false,
             string filename = "",
             double lensRadius = 0.25,
             double focalDistance = 4.7,
             std::string aperture_filename = "",
						 std::string ghost_aperture_filename = "");

  /**
   * Destructor.
   * Frees all the internal resources used by the pathtracer.
   */
  ~RaytracedRenderer();

  /**
   * If in the INIT state, configures the pathtracer to use the given scene. If
   * configuration is done, transitions to the READY state.
   * This DOES take ownership of the scene, and therefore deletes it if a new
   * scene is later passed in.
   * \param scene pointer to the new scene to be rendered
   */
  void set_scene(Scene* scene);

  /**
   * If in the INIT state, configures the pathtracer to use the given camera. If
   * configuration is done, transitions to the READY state.
   * This DOES NOT take ownership of the camera, and doesn't delete it ever.
   * \param camera the camera to use in rendering
   */
  void set_camera(Camera* camera);

  /**
   * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
   * RENDERING, or DONE), transitions to READY b/c a changing window size
   * would invalidate the output. If in INIT and configuration is done,
   * transitions to READY.
   * \param width width of the frame
   * \param height height of the frame
   */
  void set_frame_size(size_t width, size_t height);

  /**
   * Update result on screen.
   * If the pathtracer is in RENDERING or DONE, it will display the result in
   * its frame buffer. If the pathtracer is in VISUALIZE mode, it will draw
   * the BVH visualization with OpenGL.
   */
  void update_screen();

  /**
   * Transitions from any running state to READY.
   */
  void stop();

  /**
   * If the pathtracer is in READY, delete all internal data, transition to INIT.
   */
  void clear();

  /**
   * If the pathtracer is in RENDER, set the camera focal distance to the vector.
   */
  void autofocus(Vector2D loc);

  /**
   * If the pathtracer is in READY, transition to VISUALIZE.
   */
  void start_visualizing();

  /**
   * If the pathtracer is in READY, transition to RENDERING.
   */
  void start_raytracing();

  void render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy);

  void raytrace_cell(ImageBuffer& buffer);

  /**
   * If the pathtracer is in VISUALIZE, handle key presses to traverse the bvh.
   */
  void key_press(int key);

  /**
   * Save rendered result to png file.
   */
  void save_image(std::string filename="", ImageBuffer* buffer=NULL);

  /**
   * Save sampling rates to png file.
   */
  void save_sampling_rate_image(std::string filename);

 private:

  /**
   * Used in initialization.
   */
  bool has_valid_configuration();

  /**
   * Build acceleration structures.
   */
  void build_accel();

  /**
   * Visualize acceleration structures.
   */
  void visualize_accel() const;

  void visualize_cell() const;

  /**
   * Raytrace a tile of the scene and update the frame buffer. Is run
   * in a worker thread.
   */
  void raytrace_tile(int tile_x, int tile_y, int tile_w, int tile_h);

  /**
   * Implementation of a ray tracer worker thread
   */
  void worker_thread();

  enum State {
    INIT,               ///< to be initialized
    READY,              ///< initialized ready to do stuff
    VISUALIZE,          ///< visualizing BVH accelerator aggregate
    RENDERING,          ///< started but not completed raytracing
    DONE                ///< started and completed raytracing
  };

  PathTracer *pt;

  // Configurables //

  State state;          ///< current state
  Scene* scene;         ///< current scene
  Camera* camera;       ///< current camera

  // Integration state //

  vector<int> tile_samples; ///< current sample rate for tile
  size_t num_tiles_w;       ///< number of tiles along width of the image
  size_t num_tiles_h;       ///< number of tiles along height of the image

  size_t frame_w, frame_h;

  double lensRadius;
  double focalDistance;
  std::string aperture_filename;
	std::string ghost_aperture_filename;

  // Components //

  BVHAccel* bvh;                 ///< BVH accelerator aggregate
  ImageBuffer frameBuffer;       ///< frame buffer
  Timer timer;                   ///< performance test timer

  std::vector<int> sampleCountBuffer;   ///< sample count buffer

  // Internals //

  size_t numWorkerThreads;
  size_t imageTileSize;

  bool continueRaytracing;                  ///< rendering should continue
  std::vector<std::thread*> workerThreads;  ///< pool of worker threads
  std::atomic<int> workerDoneCount;         ///< worker threads management
  WorkQueue<WorkItem> workQueue;            ///< queue of work for the workers
  std::condition_variable cv_done;
  std::mutex m_done;
  size_t tilesDone;
  size_t tilesTotal;

  // Visualizer Controls //

  std::stack<BVHNode*> selectionHistory;  ///< node selection history
  std::vector<LoggedRay> rayLog;          ///< ray tracing log
  bool show_rays;                         ///< show rays from raylog
  
  std::string filename;
};

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
