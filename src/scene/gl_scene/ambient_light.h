#ifndef CGL_GLSCENE_AMBIENTLIGHT_H
#define CGL_GLSCENE_AMBIENTLIGHT_H

#include "scene.h"
#include "scene/collada/light_info.h"
#include "scene/light.h"

#include "application/visual_debugger.h"

namespace CGL { namespace GLScene {

class AmbientLight : public SceneLight {
 public:

  AmbientLight(const Collada::LightInfo& light_info) {
    this->spectrum = light_info.spectrum;
  }

  SceneObjects::SceneLight *get_static_light() const {
    SceneObjects::InfiniteHemisphereLight* l = 
      new SceneObjects::InfiniteHemisphereLight(spectrum);
    return l;
  }

  void render_debugger_node() {
    if (ImGui::TreeNode(this, "Infinite Hemisphere Light 0x%x", this))
    {
      DragDouble3("Radiance", &spectrum[0], 0.005);
      ImGui::TreePop();
    }
  }

 private:
  Vector3D spectrum;
};

} // namespace GLScene
} // namespace CGL

#endif // CGL_GLSCENE_AMBIENTLIGHT_H
