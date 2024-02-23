#pragma once
#ifndef __LAMBERTIAN_HPP__
#define __LAMBERTIAN_HPP__

#include "Shader.hpp"

namespace RayTracer {
class Lambertian : public Shader {
   private:
    Vec3 albedo;

   public:
    Lambertian(Material& material, vector<Texture>& textures);
    Scattered shade(const Ray& ray, const Vec3& hitPoint, const Vec3& normal) const;
    Vec3 eval(const Vec3& in, const Vec3& out, const Vec3& normal) const;
};
}  // namespace PathTracer

#endif