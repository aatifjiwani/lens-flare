#pragma once

#include "imgui.h"

class GLFWwindow;

namespace CGL
{

  namespace GLScene
  {
    class Scene;
  }

  class VisualDebugger
  {
  private:
    GLFWwindow* window;
    GLFWwindow* window_parent;

  public:
    bool running = true;

    GLScene::Scene** parent_scene;
    int* current_mode;

    VisualDebugger(GLScene::Scene** parent_scene, int* current_mode);
    ~VisualDebugger();

    void render();
  };

  bool DragDouble3(const char* label, const double* p_data, float v_speed);

  bool DragDouble(const char* label, const double* p_data, float v_speed);

  bool SliderDouble3(const char* label, const double* p_data, float min, float max);
}