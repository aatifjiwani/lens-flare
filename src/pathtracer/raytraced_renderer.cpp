#include "raytraced_renderer.h"
#include "bsdf.h"
#include "pathtracer/ray.h"

#include <stack>
#include <random>
#include <algorithm>
#include <sstream>

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"
#include "CGL/lodepng.h"

#include "GL/glew.h"

#include "scene/sphere.h"
#include "scene/triangle.h"
#include "scene/light.h"

using namespace CGL::SceneObjects;

using std::min;
using std::max;

namespace CGL {

/**
 * Raytraced Renderer is a render controller that in this case.
 * It controls a path tracer to produce an rendered image from the input parameters.
 *
 * A pathtracer with BVH accelerator and BVH visualization capabilities.
 * It is always in exactly one of the following states:
 * -> INIT: is missing some data needed to be usable, like a camera or scene.
 * -> READY: fully configured, but not rendering.
 * -> VISUALIZE: visualizatiNG BVH aggregate.
 * -> RENDERING: rendering a scene.
 * -> DONE: completed rendering a scene.
 */
RaytracedRenderer::RaytracedRenderer(size_t ns_aa,
                       size_t max_ray_depth, size_t ns_area_light,
                       size_t ns_diff, size_t ns_glsy, size_t ns_refr,
                       size_t num_threads,
                       size_t samples_per_batch,
                       float max_tolerance,
                       HDRImageBuffer* envmap,
                       bool direct_hemisphere_sample,
                       string filename,
                       double lensRadius,
                       double focalDistance,
                       std::string aperture_filename,
											 std::string ghost_aperature_filename) {
  state = INIT;

  pt = new PathTracer();

  pt->ns_aa = ns_aa;                                        // Number of samples per pixel
  pt->max_ray_depth = max_ray_depth;                        // Maximum recursion ray depth
  pt->ns_area_light = ns_area_light;                        // Number of samples for area light
  pt->ns_diff = ns_diff;                                    // Number of samples for diffuse surface
  pt->ns_glsy = ns_diff;                                    // Number of samples for glossy surface
  pt->ns_refr = ns_refr;                                    // Number of samples for refraction
  pt->samplesPerBatch = samples_per_batch;                  // Number of samples per batch
  pt->maxTolerance = max_tolerance;                         // Maximum tolerance for early termination
  pt->direct_hemisphere_sample = direct_hemisphere_sample;  // Whether to use direct hemisphere sampling vs. Importance Sampling

  this->lensRadius = lensRadius;
  this->focalDistance = focalDistance;
  this->aperture_filename = aperture_filename;
	this->ghost_aperture_filename = ghost_aperture_filename;

  this->filename = filename;

  if (envmap) {
    pt->envLight = new EnvironmentLight(envmap);
  } else {
    pt->envLight = NULL;
  }

  bvh = NULL;
  scene = NULL;
  camera = NULL;

  show_rays = true;

  imageTileSize = 32;                     // Size of the rendering tile.
  numWorkerThreads = num_threads;         // Number of threads
  workerThreads.resize(numWorkerThreads);
}

/**
 * Destructor.
 * Frees all the internal resources used by the pathtracer.
 */
RaytracedRenderer::~RaytracedRenderer() {

  delete bvh;
  delete pt;

}

/**
 * If in the INIT state, configures the pathtracer to use the given scene. If
 * configuration is done, transitions to the READY state.
 * This DOES take ownership of the scene, and therefore deletes it if a new
 * scene is later passed in.
 * \param scene pointer to the new scene to be rendered
 */
void RaytracedRenderer::set_scene(Scene *scene) {

  if (state != INIT) {
    return;
  }

  if (this->scene != nullptr) {
    delete scene;
    delete bvh;
    selectionHistory.pop();
  }

  if (pt->envLight != nullptr) {
    scene->lights.push_back(pt->envLight);
  }

  this->scene = scene;
  build_accel();

  if (has_valid_configuration()) {
    state = READY;
  }
}

/**
 * If in the INIT state, configures the pathtracer to use the given camera. If
 * configuration is done, transitions to the READY state.
 * This DOES NOT take ownership of the camera, and doesn't delete it ever.
 * \param camera the camera to use in rendering
 */
void RaytracedRenderer::set_camera(Camera *camera) {

  if (state != INIT) {
    return;
  }

  camera->focalDistance = focalDistance;
  camera->lensRadius = lensRadius;
  this->camera = camera;

  // Begin Lens Flare Implementation
  CameraApertureTexture* apertureTexture = new CameraApertureTexture();
  apertureTexture->init(this->aperture_filename);
	
	CameraApertureTexture* ghostApertureTexture = new CameraApertureTexture();
	ghostApertureTexture->init(this->ghost_aperture_filename);

  this->camera->aperture_texture = apertureTexture;
	this->camera->ghost_aperture_texture = ghostApertureTexture;

  if (has_valid_configuration()) {
    state = READY;
  }
}

/**
 * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
 * RENDERING, or DONE), transitions to READY b/c a changing window size
 * would invalidate the output. If in INIT and configuration is done,
 * transitions to READY.
 * \param width width of the frame
 * \param height height of the frame
 */
void RaytracedRenderer::set_frame_size(size_t width, size_t height) {
  if (state != INIT && state != READY) {
    stop();
  }

  frame_w = width;
  frame_h = height;

  frameBuffer.resize(width, height);
  cell_tl = Vector2D(0,0); 
  cell_br = Vector2D(width, height);
  render_cell = false;

  pt->set_frame_size(width, height);

  if (has_valid_configuration()) {
    state = READY;
  }
}

bool RaytracedRenderer::has_valid_configuration() {
  return scene && camera;
}

/**
 * Update result on screen.
 * If the pathtracer is in RENDERING or DONE, it will display the result in
 * its frame buffer. If the pathtracer is in VISUALIZE mode, it will draw
 * the BVH visualization with OpenGL.
 */
void RaytracedRenderer::update_screen() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      visualize_accel();
      break;
    case RENDERING:
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      if (render_cell)
        visualize_cell();
      break;
    case DONE:
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      if (render_cell)
        visualize_cell();
      break;
  }
}

/**
 * Transitions from any running state to READY.
 */
void RaytracedRenderer::stop() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      while (selectionHistory.size() > 1) {
        selectionHistory.pop();
      }
      state = READY;
      break;
    case RENDERING:
      continueRaytracing = false;
    case DONE:
      for (int i=0; i<numWorkerThreads; i++) {
            workerThreads[i]->join();
            delete workerThreads[i];
        }
      state = READY;
      break;
  }
}

/**
 * If the pathtracer is in READY, delete all internal data, transition to INIT.
 */
void RaytracedRenderer::clear() {
  if (state != READY) return;
  delete bvh;
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  selectionHistory.pop();
  frameBuffer.resize(0, 0);
  state = INIT;
  render_cell = false;

  pt->clear();
}

/**
 * If the pathtracer is in READY, transition to VISUALIZE.
 */
void RaytracedRenderer::start_visualizing() {
  if (state != READY) {
    return;
  }
  state = VISUALIZE;
}

/**
 * If the pathtracer is in READY, transition to RENDERING.
 */
void RaytracedRenderer::start_raytracing() {
  if (state != READY) return;

  rayLog.clear();
  workQueue.clear();

  state = RENDERING;
  continueRaytracing = true;
  workerDoneCount = 0;

  size_t width = frameBuffer.w;
  size_t height = frameBuffer.h;

  pt->clear();
  pt->set_frame_size(width, height);

  pt->bvh = bvh;
  pt->camera = camera;
  pt->scene = scene;
  pt->find_sun_pos();
	
	// generate ghost_buffer
	pt->gen_ghost_buffer();
	

  if (!render_cell) {
    frameBuffer.clear();
    num_tiles_w = width / imageTileSize + 1;
    num_tiles_h = height / imageTileSize + 1;
    tilesTotal = num_tiles_w * num_tiles_h;
    tilesDone = 0;
    tile_samples.resize(num_tiles_w * num_tiles_h);
    memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

    // populate the tile work queue
    for (size_t y = 0; y < height; y += imageTileSize) {
        for (size_t x = 0; x < width; x += imageTileSize) {
            workQueue.put_work(WorkItem(x, y, imageTileSize, imageTileSize));
        }
    }
  } else {
    int w = (cell_br-cell_tl).x;
    int h = (cell_br-cell_tl).y;
    int imTS = imageTileSize / 4;
    num_tiles_w = w / imTS + 1;
    num_tiles_h = h / imTS + 1;
    tilesTotal = num_tiles_w * num_tiles_h;
    tilesDone = 0;
    tile_samples.resize(num_tiles_w * num_tiles_h);
    memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

    // populate the tile work queue
    for (size_t y = cell_tl.y; y < cell_br.y; y += imTS) {
      for (size_t x = cell_tl.x; x < cell_br.x; x += imTS) {
        workQueue.put_work(WorkItem(x, y, 
          min(imTS, (int)(cell_br.x-x)), min(imTS, (int)(cell_br.y-y)) ));
      }
    }
  }

  bvh->total_isects = 0; bvh->total_rays = 0;
  // launch threads
  fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);
  for (int i=0; i<numWorkerThreads; i++) {
      workerThreads[i] = new std::thread(&RaytracedRenderer::worker_thread, this);
  }
}

void RaytracedRenderer::render_to_file(string filename, size_t x, size_t y, size_t dx, size_t dy) {
  if (x == -1) {
    unique_lock<std::mutex> lk(m_done);
    start_raytracing();
    cv_done.wait(lk, [this]{ return state == DONE; });
    lk.unlock();
    save_image(filename);
    fprintf(stdout, "[PathTracer] Job completed.\n");
  } else {
    render_cell = true;
    cell_tl = Vector2D(x,y);
    cell_br = Vector2D(x+dx,y+dy);
    ImageBuffer buffer;
    raytrace_cell(buffer);
    save_image(filename, &buffer);
    fprintf(stdout, "[PathTracer] Cell job completed.\n");
  }
}


void RaytracedRenderer::build_accel() {

  // collect primitives //
  fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
  timer.start();
  vector<Primitive *> primitives;
  for (SceneObject *obj : scene->objects) {
    const vector<Primitive *> &obj_prims = obj->get_primitives();
    primitives.reserve(primitives.size() + obj_prims.size());
    primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
  }
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // build BVH //
  fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", primitives.size()); 
  fflush(stdout);
  timer.start();
  bvh = new BVHAccel(primitives);
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // initial visualization //
  selectionHistory.push(bvh->get_root());
}

void RaytracedRenderer::visualize_accel() const {

  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);
  glEnable(GL_DEPTH_TEST);

  // hardcoded color settings
  Color cnode = Color(.5, .5, .5); float cnode_alpha = 0.25f;
  Color cnode_hl = Color(1., .25, .0); float cnode_hl_alpha = 0.6f;
  Color cnode_hl_child = Color(1., 1., 1.); float cnode_hl_child_alpha = 0.6f;

  Color cprim_hl_left = Color(.6, .6, 1.); float cprim_hl_left_alpha = 1.f;
  Color cprim_hl_right = Color(.8, .8, 1.); float cprim_hl_right_alpha = 1.f;
  Color cprim_hl_edges = Color(0., 0., 0.); float cprim_hl_edges_alpha = 0.5f;

  BVHNode *selected = selectionHistory.top();

  // render solid geometry (with depth offset)
  glPolygonOffset(1.0, 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);

  if (selected->isLeaf()) {
    bvh->draw(selected, cprim_hl_left, cprim_hl_left_alpha);
  } else {
    bvh->draw(selected->l, cprim_hl_left, cprim_hl_left_alpha);
    bvh->draw(selected->r, cprim_hl_right, cprim_hl_right_alpha);
  }

  glDisable(GL_POLYGON_OFFSET_FILL);

  // draw geometry outline
  bvh->drawOutline(selected, cprim_hl_edges, cprim_hl_edges_alpha);

  // keep depth buffer check enabled so that mesh occluded bboxes, but
  // disable depth write so that bboxes don't occlude each other.
  glDepthMask(GL_FALSE);

  // create traversal stack
  stack<BVHNode *> tstack;

  // push initial traversal data
  tstack.push(bvh->get_root());

  // draw all BVH bboxes with non-highlighted color
  while (!tstack.empty()) {

    BVHNode *current = tstack.top();
    tstack.pop();

    current->bb.draw(cnode, cnode_alpha);
    if (current->l) tstack.push(current->l);
    if (current->r) tstack.push(current->r);
  }

  // draw selected node bbox and primitives
  if (selected->l) selected->l->bb.draw(cnode_hl_child, cnode_hl_child_alpha);
  if (selected->r) selected->r->bb.draw(cnode_hl_child, cnode_hl_child_alpha);

  glLineWidth(3.f);
  selected->bb.draw(cnode_hl, cnode_hl_alpha);

  // now perform visualization of the rays
  if (show_rays) {
      glLineWidth(1.f);
      glBegin(GL_LINES);

      for (size_t i=0; i<rayLog.size(); i+=500) {

          const static double VERY_LONG = 10e4;
          double ray_t = VERY_LONG;

          // color rays that are hits yellow
          // and rays this miss all geometry red
          if (rayLog[i].hit_t >= 0.0) {
              ray_t = rayLog[i].hit_t;
              glColor4f(1.f, 1.f, 0.f, 0.1f);
          } else {
              glColor4f(1.f, 0.f, 0.f, 0.1f);
          }

          Vector3D end = rayLog[i].o + ray_t * rayLog[i].d;

          glVertex3f(rayLog[i].o[0], rayLog[i].o[1], rayLog[i].o[2]);
          glVertex3f(end[0], end[1], end[2]);
      }
      glEnd();
  }

  glDepthMask(GL_TRUE);
  glPopAttrib();
}

void RaytracedRenderer::visualize_cell() const {
  glPushAttrib(GL_VIEWPORT_BIT);
  glViewport(0, 0, frameBuffer.w, frameBuffer.h);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, frameBuffer.w, frameBuffer.h, 0, 0, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(0, 0, -1);

  glColor4f(1.0, 0.0, 0.0, 0.8);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  // Draw the Red Rectangle.
  glBegin(GL_LINE_LOOP);
  glVertex2f(cell_tl.x, frameBuffer.h-cell_br.y);
  glVertex2f(cell_br.x, frameBuffer.h-cell_br.y);
  glVertex2f(cell_br.x, frameBuffer.h-cell_tl.y);
  glVertex2f(cell_tl.x, frameBuffer.h-cell_tl.y);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glPopAttrib();

  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
}

/**
 * If the pathtracer is in VISUALIZE, handle key presses to traverse the bvh.
 */
void RaytracedRenderer::key_press(int key) {
  BVHNode *current = selectionHistory.top();
  switch (key) {
  case ']':
    pt->ns_aa *=2;
    fprintf(stdout, "[PathTracer] Samples per pixel changed to %lu\n", pt->ns_aa);
    //tm_key = clamp(tm_key + 0.02f, 0.0f, 1.0f);
    break;
  case '[':
    //tm_key = clamp(tm_key - 0.02f, 0.0f, 1.0f);
    pt->ns_aa /=2;
    if (pt->ns_aa < 1) pt->ns_aa = 1;
    fprintf(stdout, "[PathTracer] Samples per pixel changed to %lu\n", pt->ns_aa);
    break;
  case '=': case '+':
    pt->ns_area_light *= 2;
    fprintf(stdout, "[PathTracer] Area light sample count increased to %zu.\n", pt->ns_area_light);
    break;
  case '-': case '_':
    if (pt->ns_area_light > 1) pt->ns_area_light /= 2;
    fprintf(stdout, "[PathTracer] Area light sample count decreased to %zu.\n", pt->ns_area_light);
    break;
  case '.': case '>':
    pt->max_ray_depth++;
    fprintf(stdout, "[PathTracer] Max ray depth increased to %zu.\n", pt->max_ray_depth);
    break;
  case ',': case '<':
    if (pt->max_ray_depth) pt->max_ray_depth--;
    fprintf(stdout, "[PathTracer] Max ray depth decreased to %zu.\n", pt->max_ray_depth);
    break;
  case 'h': case 'H':
    pt->direct_hemisphere_sample = !pt->direct_hemisphere_sample;
    fprintf(stdout, "[PathTracer] Toggled direct lighting to %s\n", (pt->direct_hemisphere_sample ? "uniform hemisphere sampling" : "importance light sampling"));
    break;
  case 'k': case 'K':
    pt->camera->lensRadius = std::max(pt->camera->lensRadius - 0.05, 0.0);
    fprintf(stdout, "[PathTracer] Camera lens radius reduced to %f.\n", pt->camera->lensRadius);
    break;
  case 'l': case 'L':
    pt->camera->lensRadius = pt->camera->lensRadius + 0.05;
    fprintf(stdout, "[PathTracer] Camera lens radius increased to %f.\n", pt->camera->lensRadius);
    break;
  case ';':
    pt->camera->focalDistance = std::max(pt->camera->focalDistance - 0.1, 0.0);
    fprintf(stdout, "[PathTracer] Camera focal distance reduced to %f.\n", pt->camera->focalDistance);
    break;
  case '\'':
    pt->camera->focalDistance = pt->camera->focalDistance + 0.1;
    fprintf(stdout, "[PathTracer] Camera focal distance increased to %f.\n", pt->camera->focalDistance);
    break;
  case KEYBOARD_UP:
    if (current != bvh->get_root()) {
        selectionHistory.pop();
    }
    break;
  case KEYBOARD_LEFT:
    if (current->l) {
        selectionHistory.push(current->l);
    }
    break;
  case KEYBOARD_RIGHT:
    if (current->l) {
        selectionHistory.push(current->r);
    }
    break;

  case 'C':
    render_cell = !render_cell;
    if (render_cell)
      fprintf(stdout, "[PathTracer] Now in cell render mode.\n");
    else
      fprintf(stdout, "[PathTracer] No longer in cell render mode.\n");
    break;

  case 'a': case 'A':
    show_rays = !show_rays;
  default:
    return;
  }
}

/**
 * Raytrace a tile of the scene and update the frame buffer. Is run
 * in a worker thread.
 */
void RaytracedRenderer::raytrace_tile(int tile_x, int tile_y,
                               int tile_w, int tile_h) {
  size_t w = frame_w;
  size_t h = frame_h;

  size_t tile_start_x = tile_x;
  size_t tile_start_y = tile_y;

  size_t tile_end_x = std::min(tile_start_x + tile_w, w);
  size_t tile_end_y = std::min(tile_start_y + tile_h, h);

  size_t tile_idx_x = tile_x / imageTileSize;
  size_t tile_idx_y = tile_y / imageTileSize;
  size_t num_samples_tile = tile_samples[tile_idx_x + tile_idx_y * num_tiles_w];

  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    if (!continueRaytracing) return;
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
      pt->raytrace_pixel(x, y);
    }
  }

  tile_samples[tile_idx_x + tile_idx_y * num_tiles_w] += 1;

  pt->write_to_framebuffer(frameBuffer, tile_start_x, tile_start_y, tile_end_x, tile_end_y);
}

void RaytracedRenderer::raytrace_cell(ImageBuffer& buffer) {
  size_t tile_start_x = cell_tl.x;
  size_t tile_start_y = cell_tl.y;

  size_t tile_end_x = cell_br.x;
  size_t tile_end_y = cell_br.y;

  size_t w = tile_end_x - tile_start_x;
  size_t h = tile_end_y - tile_start_y;
  HDRImageBuffer sb(w, h);
  buffer.resize(w,h);

  stop();
  render_cell = true;
  {
    unique_lock<std::mutex> lk(m_done);
    start_raytracing();
    cv_done.wait(lk, [this]{ return state == DONE; });
    lk.unlock();
  }

  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
      buffer.data[w*(y-tile_start_y)+(x-tile_start_x)] = frameBuffer.data[x+y*frame_w];
    }
  }
}

void RaytracedRenderer::autofocus(Vector2D loc) {
  pt->autofocus(loc);
}

void RaytracedRenderer::worker_thread() {

  Timer timer;
  timer.start();

  WorkItem work;
  while (continueRaytracing && workQueue.try_get_work(&work)) {
    raytrace_tile(work.tile_x, work.tile_y, work.tile_w, work.tile_h);
    { 
      lock_guard<std::mutex> lk(m_done);
      ++tilesDone;
      cout << "\r[PathTracer] Rendering... " << int((double)tilesDone/tilesTotal * 100) << '%';
      cout.flush();
    }
  }

  workerDoneCount++;
  if (!continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    fprintf(stdout, "\n[PathTracer] Rendering canceled!\n");
    state = READY;
  }

  if (continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    fprintf(stdout, "\r[PathTracer] Rendering... 100%%! (%.4fs)\n", timer.duration());
    fprintf(stdout, "[PathTracer] BVH traced %llu rays.\n", bvh->total_rays);
    fprintf(stdout, "[PathTracer] Average speed %.4f million rays per second.\n", (double)bvh->total_rays / timer.duration() * 1e-6);
    fprintf(stdout, "[PathTracer] Averaged %f intersection tests per ray.\n", (((double)bvh->total_isects)/bvh->total_rays));

    lock_guard<std::mutex> lk(m_done);
    state = DONE;
    cv_done.notify_one();
  }
}

void RaytracedRenderer::save_image(string filename, ImageBuffer* buffer) {

  if (state != DONE) return;

  if (!buffer)
    buffer = &frameBuffer;

  if (filename == "") {
    time_t rawtime;
    time (&rawtime);

    time_t t = time(nullptr);
    tm *lt = localtime(&t);
    stringstream ss;
    ss << this->filename << "_screenshot_" << lt->tm_mon+1 << "-" << lt->tm_mday << "_" 
      << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".png";
    filename = ss.str();  
  }

  uint32_t* frame = &buffer->data[0];
  size_t w = buffer->w;
  size_t h = buffer->h;
  uint32_t* frame_out = new uint32_t[w * h];
  for(size_t i = 0; i < h; ++i) {
    memcpy(frame_out + i * w, frame + (h - i - 1) * w, 4 * w);
  }
  
  for (size_t i = 0; i < w * h; ++i) {
    frame_out[i] |= 0xFF000000;
  }

  fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());
  lodepng::encode(filename, (unsigned char*) frame_out, w, h);
  fprintf(stderr, "Done!\n");
  
  delete[] frame_out;

  save_sampling_rate_image(filename);
}

void RaytracedRenderer::save_sampling_rate_image(string filename) {
  size_t w = frameBuffer.w;
  size_t h = frameBuffer.h;
  ImageBuffer outputBuffer(w, h);

  for (int x = 0; x < w; x++) {
      for (int y = 0; y < h; y++) {
          float samplingRate = pt->sampleCountBuffer[y * w + x] * 1.0f / pt->ns_aa;

          Color c;
          if (samplingRate <= 0.5) {
              float r = (0.5 - samplingRate) / 0.5;
              c = Color(0.0f, 0.0f, 1.0f) * r + Color(0.0f, 1.0f, 0.0f) * (1.0 - r);
          } else {
              float r = (1.0 - samplingRate) / 0.5;
              c = Color(0.0f, 1.0f, 0.0f) * r + Color(1.0f, 0.0f, 0.0f) * (1.0 - r);
          }
          outputBuffer.update_pixel(c, x, h - 1 - y);
      }
  }
  uint32_t* frame_out = new uint32_t[w * h];
  
  for (size_t i = 0; i < w * h; ++i) {
    uint32_t out_color_hex = 0;
    frame_out[i] = outputBuffer.data.data()[i];
    frame_out[i] |= 0xFF000000;
  }

  lodepng::encode(filename.substr(0,filename.size()-4) + "_rate.png", (unsigned char*) frame_out, w, h);
  
  delete[] frame_out;
}

}  // namespace CGL
