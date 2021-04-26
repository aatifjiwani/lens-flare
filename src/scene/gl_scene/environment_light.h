#ifndef CGL_GLSCENE_ENVIRONMENTLIGHT_H
#define CGL_GLSCENE_ENVIRONMENTLIGHT_H

#include "scene.h"
#include "util/image.h"
#include "scene/light.h"

#include "application/visual_debugger.h"

namespace CGL { namespace GLScene {

class EnvironmentLight : public SceneLight {
 public:

  EnvironmentLight(HDRImageBuffer* envmap) : envmap(envmap) { }

  SceneObjects::SceneLight *get_static_light() const {
    SceneObjects::EnvironmentLight* l = 
      new SceneObjects::EnvironmentLight(envmap);
    return l;
  }

  void render_debugger_node() {
    if (ImGui::TreeNode(this, "Environment Light 0x%x", this))
    {
      ImGui::Text("HDRI Image Buffer (%dx%d): 0x%x", envmap->w, envmap->h, envmap);
      ImGui::TreePop();
    }
  }

 private:

  HDRImageBuffer* envmap;

};

} // namespace GLScene
} // namespace CGL

#endif // CGL_GLSCENE_ENVIRONMENTLIGHT_H
