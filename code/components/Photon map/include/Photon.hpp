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
#include<map>
namespace PhotonMap {
using namespace NRenderer;
using namespace std;

enum reflectionType {
    diffusion = 0,
    specular,
    absorb
};

class nearestNeighbor
{
public:
    Vec3 pos;
    int maxPhoton;
    int found;
    int sample;
    float l2dis;
    //Photon photons[100];
    float photonDis[100];
    bool gotHeap;
    map<float, Photon> photonDict;
    nearestNeighbor(Vec3 v)
    {
        sample = 15;
        pos = v;
        found = 0;
        maxPhoton = 100;
        l2dis = 500;//500
        gotHeap = false;
    }

    void getNearest(photonMap* phtmap);
    void getNearestWithKD(photonMap* phtmap, int index);
};

class PathTracerRenderer {
   public:
   private:
    default_random_engine eng;
    SharedScene spScene;
    Scene& scene;

    unsigned int width;
    unsigned int height;
    unsigned int depth;
    unsigned int samples;

    float russian;
    photonMap* phtmap;

    using SCam = PhotonMap::Camera;
    SCam camera;

    vector<SharedShader> shaderPrograms;

   public:
    PathTracerRenderer(SharedScene spScene) : spScene(spScene), scene(*spScene), camera(spScene->camera)
    {
        width = scene.renderOption.width;
        height = scene.renderOption.height;
        depth = scene.renderOption.depth;
        samples = scene.renderOption.samplesPerPixel;
        russian = scene.renderOption.russian;
        phtmap = new photonMap(100000);
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
    HitRecord closestHitObject(const Photon& p);
    HitRecord defaultLightSample() const;
    HitRecord BVHTree(const Ray& ray, BVHBuildNode* node);
    HitRecord xMesh(const Ray& ray, const Mesh& a, float tMin = 0.f, float tMax = FLOAT_INF);
    bool xBounds(const Ray& ray, const Bounds& b);//只能放这了。。。。
    HitRecord theClosestHitObject(BVH* bvh, const Ray& ray);
    tuple<float, Vec3> closestHitLight(const Ray& r);

    public:
     void generatePhoton(AreaLight a, Vec3& ori, float& PowerScale, Vec3& dir);
     void generatePhotonMap();
     void tracePhoton(Photon& p, int depth);
     RGB getIrradiance(Ray r);
     //int russian(float dif);
};
}  // namespace PathTracer

#endif