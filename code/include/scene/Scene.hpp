#pragma once
#ifndef __NR_SCENE_HPP__
#define __NR_SCENE_HPP__

#include "../app/include/manager/RenderSettingsManager.hpp"
#include "BVH.hpp"
#include "Camera.hpp"
#include "Light.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "Model.hpp"
#include "Texture.hpp"

namespace NRenderer
{
struct RenderOption
{
    unsigned int width;
    unsigned int height;
    unsigned int depth;
    unsigned int samplesPerPixel;
    float russian;
    RenderOption()
        : width(500),
          height(500),
          depth(4),
          samplesPerPixel(16),
          russian(0.8){}
};

struct Ambient
{
    enum class Type { CONSTANT, ENVIROMENT_MAP };
    Type type;
    Vec3 constant = {};
    Handle environmentMap = {};
};

struct Scene
{
    Camera camera;

    RenderOption renderOption;

    Ambient ambient;

    // buffers
    vector<Material> materials;
    vector<Texture> textures;

    vector<Model> models;
    vector<Node> nodes;
    // object buffer
    vector<Sphere> sphereBuffer;
    vector<Triangle> triangleBuffer;
    vector<Plane> planeBuffer;
    vector<Mesh> meshBuffer;

    vector<Light> lights;
    // light buffer
    vector<PointLight> pointLightBuffer;
    vector<AreaLight> areaLightBuffer;
    vector<DirectionalLight> directionalLightBuffer;
    vector<SpotLight> spotLightBuffer;

    BVH* bvh;
    void buildBVHTree()
    {
        vector<Entity*> primitives;
        for (auto& s : sphereBuffer)
            primitives.push_back(&s);
        for (auto& t : triangleBuffer)
            primitives.push_back(&t);
        for (auto& p : planeBuffer)
            primitives.push_back(&p);
        for (auto& m : meshBuffer) {
            m.buildBVH();
            primitives.push_back(&m);
        }
        bvh = new BVH(primitives);
    }
};
using SharedScene = shared_ptr<Scene>;
}  // namespace NRenderer

#endif