#include "Camera.hpp"
#include "component/RenderComponent.hpp"
#include "scene/Scene.hpp"
#include "server/Server.hpp"

#include "RayTracer.hpp"

using namespace std;
using namespace NRenderer;

namespace RayTracer {
class Adapter : public RenderComponent {
    void render(SharedScene spScene) {
        PathTracerRenderer renderer{spScene};
        auto renderResult = renderer.render();
        auto [pixels, width, height] = renderResult;
        getServer().screen.set(pixels, width, height);
        renderer.release(renderResult);
    }
};
}  // namespace PathTracer

const static string description =
    "A Path Tracer. "
    "Only some simple primitives and materials(Lambertian) are supported."
    "\nPlease use scene file : cornel_area_light.scn\n";

REGISTER_RENDERER(RayTracer, description, RayTracer::Adapter);