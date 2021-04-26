#ifndef CGL_GLSCENE_SPOTLIGHT_H
#define CGL_GLSCENE_SPOTLIGHT_H

#include "scene.h"
#include "scene/light.h"

#include "application/visual_debugger.h"

namespace CGL { namespace GLScene {

class SpotLight : public SceneLight {
 public:

  SpotLight(const Collada::LightInfo& light_info, 
           const Matrix4x4& transform) {

    this->spectrum = light_info.spectrum;
    this->position = (transform * Vector4D(light_info.position, 1)).to3D();
    this->direction = (transform * Vector4D(light_info.direction, 1)).to3D() - position;
    this->direction.normalize();
  }

  SceneObjects::SceneLight *get_static_light() const {
    SceneObjects::SpotLight* l = 
      new SceneObjects::SpotLight(spectrum, position, direction, PI * .5f);
    return l;
  }

  void render_debugger_node() {
    if (ImGui::TreeNode(this, "Spot Light 0x%x", this))
    {
      DragDouble3("Radiance", &spectrum[0], 0.005);
      DragDouble3("Position", &position[0], 0.005);
      DragDouble3("Direction", &direction[0], 0.005);
      ImGui::TreePop();
    }
  }

 private:

  Vector3D spectrum;
  Vector3D direction;
  Vector3D position;

};

} // namespace GLScene
} // namespace CGL

#endif //CGL_GLSCENE_SPOTLIGHT_H
