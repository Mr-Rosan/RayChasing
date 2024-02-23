#pragma once
#ifndef __SIMPLE_PATH_TRACER_HPP__
#define __SIMPLE_PATH_TRACER_HPP__

#include "Camera.hpp"
#include "Ray.hpp"
#include "intersections/HitRecord.hpp"
#include "scene/Scene.hpp"

#include "../app/include/manager/RenderSettingsManager.hpp"
#include "shaders/ShaderCreator.hpp"

#include <tuple>
namespace RayTracer {
using namespace NRenderer;
using namespace std;

class PathTracerRenderer {
   public:
   private:
    SharedScene spScene;
    Scene& scene;

    unsigned int width;
    unsigned int height;
    unsigned int depth;
    unsigned int samples;

    float russian;

    using SCam = RayTracer::Camera;
    SCam camera;

    vector<SharedShader> shaderPrograms;

   public:
    PathTracerRenderer(SharedScene spScene) : spScene(spScene), scene(*spScene), camera(spScene->camera) {
        width = scene.renderOption.width;
        height = scene.renderOption.height;
        depth = scene.renderOption.depth;
        samples = scene.renderOption.samplesPerPixel;
        russian = scene.renderOption.russian;
    }
    ~PathTracerRenderer() = default;

    using RenderResult = tuple<RGBA*, unsigned int, unsigned int>;
    RenderResult render();
    void release(const RenderResult& r);

   private:
    void renderTask(RGBA* pixels, int width, int height, int off, int step);

    RGB gamma(const RGB& rgb);
    RGB trace(const Ray& ray, int currDepth);
    HitRecord closestHitObject(const Ray& r);
    HitRecord closestHitObjectWithoutBVH(const Ray& r);
    HitRecord defaultLightSample() const;
    HitRecord BVHTree(const Ray& ray, BVHBuildNode* node);
    HitRecord xMesh(const Ray& ray, const Mesh& a, float tMin = 0.f, float tMax = FLOAT_INF);
    bool xBounds(const Ray& ray, const Bounds& b);//只能放这了。。。。
    HitRecord theClosestHitObject(BVH* bvh, const Ray& ray);
    tuple<float, Vec3> closestHitLight(const Ray& r);
};
}  // namespace PathTracer

#endif