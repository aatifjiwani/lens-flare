#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include <stdio.h>

#include "visual_debugger.h"

#include "application/application.h"
#include "application/renderer.h"
#include "pathtracer/intersection.h"
#include "pathtracer/pathtracer.h"
#include "scene/scene.h"
#include "scene/light.h"
#include "scene/triangle.h"
#include "scene/sphere.h"
#include "scene/gl_scene/mesh.h"
#include "scene/gl_scene/sphere.h"
#include "scene/gl_scene/ambient_light.h"
#include "scene/gl_scene/area_light.h"
#include "scene/gl_scene/directional_light.h"
#include "scene/gl_scene/environment_light.h"

#include <GLFW/glfw3.h>

// #define ENABLE_VISUAL_DEBUGGER

namespace CGL
{

  static void glfw_error_callback(int error, const char* description)
  {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
  }

  bool DragDouble3(const char* label, const double* p_data, float v_speed)
  {
    return ImGui::DragScalarN(label, ImGuiDataType_Double, (void*)p_data, 3, v_speed);
  }

  bool DragDouble(const char* label, const double* p_data, float v_speed)
  {
    return ImGui::DragScalar(label, ImGuiDataType_Double, (void*)p_data, v_speed);
  }

  bool SliderDouble3(const char* label, const double* p_data, float min, float max)
  {
    return ImGui::SliderScalarN(label, ImGuiDataType_Double, (void*)p_data, 3, &min, &max, "%f");
  }

  void test_light(SceneObjects::SceneLight* l)
  {
    static double distToLight, pdf;
    static Vector3D p;
    static Vector3D wi;

    DragDouble3("Sample position", &p[0], 0.005);

    if (ImGui::Button("Sample!"))
      l->sample_L(p, &wi, &distToLight, &pdf);

    ImGui::Text("sample_L(p = {%f, %f, %f}, &wi, &distToLight, &pdf);", p[0], p[1], p[2]);
    ImGui::Text("wi = {%f, %f, %f}", wi[0], wi[1], wi[2]);
    ImGui::Text("distToLight = %f", distToLight);
    ImGui::Text("pdf = %f", pdf);
  }

  VisualDebugger::VisualDebugger(GLScene::Scene** parent_scene, int* current_mode)
  {
#ifdef ENABLE_VISUAL_DEBUGGER
    this->parent_scene = parent_scene;
    this->current_mode = current_mode;

    this->window_parent = glfwGetCurrentContext();

    // Setup window
    window = glfwCreateWindow(800, 600, "Visual Debugger", NULL, NULL);
    if (window == NULL) return;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();
  
    glfwMakeContextCurrent(window_parent);
#endif
  }

  void VisualDebugger::render()
  {
#ifdef ENABLE_VISUAL_DEBUGGER
    glfwMakeContextCurrent(window);

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    GLScene::Scene* scene = *parent_scene;

    int window_x, window_y;
    glfwGetWindowSize(window, &window_x, &window_y);

    ImGui::SetNextWindowSize(ImVec2(window_x, window_y));
    ImGui::SetNextWindowPos(ImVec2(0, 0));

    ImGui::Begin("Debugger", 0, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);

    if (ImGui::BeginTabBar("Visual Debugger", ImGuiTabBarFlags_None))
    {
      if (ImGui::BeginTabItem("Scene"))
      {
        // Scene
        if (scene != nullptr && *current_mode == Application::EDIT_MODE)
        {
          scene->render_debugger_node();
        }
        else
        {
          ImGui::Text("This panel only works in EDIT mode (E)");
        }

        ImGui::EndTabItem();
      }

      // sample_L tester
      if (ImGui::BeginTabItem("`sample_L` Tester"))
      {
        if (scene != nullptr)
        {
          for (auto e : scene->lights)
          {
            if (ImGui::TreeNode(e, "Light 0x%x", e))
            {
              test_light(e->get_static_light());
              ImGui::TreePop();
            }
          }
        }

        ImGui::EndTabItem();
      }

      // Intersection Tester
      if (ImGui::BeginTabItem("Intersection Tester"))
      {
        static Ray r;

        ImGui::Text("Ray");
        DragDouble3("Origin", &r.o[0], 0.005);
        DragDouble3("Direction", &r.d[0], 0.005);
        DragDouble("Min T", &r.min_t, 0.005);
        DragDouble("Max T", &r.max_t, 0.005);

        if (ImGui::TreeNode("Triangle"))
        {
          static SceneObjects::Triangle t;

          DragDouble3("P1", &t.p1[0], 0.005);
          DragDouble3("P2", &t.p2[0], 0.005);
          DragDouble3("P3", &t.p3[0], 0.005);

          DragDouble3("N1", &t.n1[0], 0.005);
          DragDouble3("N2", &t.n2[0], 0.005);
          DragDouble3("N3", &t.n3[0], 0.005);

          static SceneObjects::Intersection isect;

          static bool success = false;

          if (ImGui::Button("Test Intersect"))
          {
            success = t.intersect(r, &isect);
          }

          if (success)
          {
            ImGui::Text("Intersection at t=%f", isect.t);
            ImGui::Text("Normal = {%f, %f, %f}", isect.n.x, isect.n.y, isect.n.z);
          }
          else
          {
            ImGui::Text("No intersection");
          }

          ImGui::TreePop();
        }

        if (ImGui::TreeNode("Sphere"))
        {
          static SceneObjects::Sphere s;

          SceneObjects::SphereObject so(Vector3D(0.0), 0.0, nullptr);
          s.object = &so;

          DragDouble3("Origin", &s.o[0], 0.005);
          DragDouble("Radius", &s.r, 0.005);

          s.r2 = s.r * s.r;

          static SceneObjects::Intersection isect;

          static bool success = false;

          if (ImGui::Button("Test Intersect"))
          {
            success = s.intersect(r, &isect);
          }

          if (success)
          {
            ImGui::Text("Intersection at t=%f", isect.t);
            ImGui::Text("Normal = {%f, %f, %f}", isect.n.x, isect.n.y, isect.n.z);
          }
          else
          {
            ImGui::Text("No intersection");
          }

          ImGui::TreePop();
        }

        ImGui::EndTabItem();
      }

      ImGui::EndTabBar();
    }

    ImGui::End();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.2, 0.6, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);

    glfwMakeContextCurrent(window_parent);
#endif
  }

  VisualDebugger::~VisualDebugger()
  {
#ifdef ENABLE_VISUAL_DEBUGGER
    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
#endif
  }

}