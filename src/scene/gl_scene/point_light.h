#ifndef CGL_GLSCENE_POINTLIGHT_H
#define CGL_GLSCENE_POINTLIGHT_H

#include "scene.h"
#include "scene/light.h"

#include "application/visual_debugger.h"

using std::cout;
using std::endl;

namespace CGL { namespace GLScene {

class PointLight : public SceneLight {
 public:

  PointLight(const Collada::LightInfo& light_info, 
             const Matrix4x4& transform) {    

    this->spectrum = light_info.spectrum;
    this->position = (transform * Vector4D(light_info.position, 1)).to3D();
  }

  SceneObjects::SceneLight *get_static_light() const {
    SceneObjects::PointLight *l = 
      new SceneObjects::PointLight(spectrum, position);
    return l;
  }

  void render_debugger_node() {
    if (ImGui::TreeNode(this, "Point Light 0x%x", this))
    {
      DragDouble3("Radiance", &spectrum[0], 0.005);
      DragDouble3("Position", &position[0], 0.005);
      ImGui::TreePop();
    }
  }

 private:

  Vector3D spectrum;
  Vector3D position;

};

} // namespace GLScene
} // namespace CGL

#endif //CGL_GLSCENE_POINTLIGHT_H
