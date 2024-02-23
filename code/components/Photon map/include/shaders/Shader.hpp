#pragma once
#ifndef __SHADER_HPP__
#define __SHADER_HPP__

#include "geometry/vec.hpp"
#include "common/macros.hpp"
#include "scene/Scene.hpp"

#include "Scattered.hpp"

namespace PhotonMap {
using namespace NRenderer;
using namespace std;

constexpr float PI = 3.1415926535898f;

class Shader {
   public:
    Material& material;
    vector<Texture>& textureBuffer;

    Shader(Material& material, vector<Texture>& textures)
        : material(material), textureBuffer(textures) {}
    virtual Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const = 0;
    virtual Vec3 eval(const Vec3& in, const Vec3& out, const Vec3& normal) const = 0;
};
SHARE(Shader);
}  // namespace PathTracer

#endif