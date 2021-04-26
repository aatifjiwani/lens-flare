#pragma once

#include "pathtracer/camera.h"
#include "util/image.h"
#include "util/work_queue.h"

#include "application/renderer.h"

#include "scene/scene.h"
using CGL::SceneObjects::Scene;

namespace CGL {

class OfflineRenderer {
public:
    virtual ~OfflineRenderer() {}

    /**
     * If in the INIT state, configures the pathtracer to use the given scene. If
     * configuration is done, transitions to the READY state.
     * This DOES take ownership of the scene, and therefore deletes it if a new
     * scene is later passed in.
     * \param scene pointer to the new scene to be rendered
     */
    virtual void set_scene(Scene* scene) = 0;

    /**
     * If in the INIT state, configures the pathtracer to use the given camera. If
     * configuration is done, transitions to the READY state.
     * This DOES NOT take ownership of the camera, and doesn't delete it ever.
     * \param camera the camera to use in rendering
     */
    virtual void set_camera(Camera* camera) = 0;

    /**
     * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
     * RENDERING, or DONE), transitions to READY b/c a changing window size
     * would invalidate the output. If in INIT and configuration is done,
     * transitions to READY.
     * \param width width of the frame
     * \param height height of the frame
     */
    virtual void set_frame_size(size_t width, size_t height) = 0;

    /**
     * Update result on screen.
     * If the pathtracer is in RENDERING or DONE, it will display the result in
     * its frame buffer. If the pathtracer is in VISUALIZE mode, it will draw
     * the BVH visualization with OpenGL.
     */
    virtual void update_screen() = 0;

    /**
     * Transitions from any running state to READY.
     */
    virtual void stop() = 0;

    /**
     * If the pathtracer is in READY, delete all internal data, transition to INIT.
     */
    virtual void clear() = 0;

    /**
     * If the pathtracer is in RENDER, set the camera focal distance to the vector.
     */
    virtual void autofocus(Vector2D loc) = 0;

    /**
     * If the pathtracer is in READY, transition to VISUALIZE.
     */
    virtual void start_visualizing() = 0;

    /**
     * If the pathtracer is in READY, transition to RENDERING.
     */
    virtual void start_raytracing() = 0;

    virtual void render_to_file(std::string filename, size_t x, size_t y, size_t dx, size_t dy) = 0;

    virtual void raytrace_cell(ImageBuffer& buffer) = 0;

    /**
     * If the pathtracer is in VISUALIZE, handle key presses to traverse the bvh.
     */
    virtual void key_press(int key) = 0;

    /**
     * Save rendered result to png file.
     */
    virtual void save_image(std::string filename = "", ImageBuffer* buffer = NULL) = 0;

    /**
     * Save sampling rates to png file.
     */
    virtual void save_sampling_rate_image(std::string filename) = 0;

    Vector2D cell_tl, cell_br;
    bool render_cell;
};


}