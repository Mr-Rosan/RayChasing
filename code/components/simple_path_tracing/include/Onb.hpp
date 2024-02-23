#pragma once
#ifndef __ONB_HPP__
#define __ONB_HPP__

#include "geometry/vec.hpp"

namespace SimplePathTracer {
using namespace NRenderer;
// 正交基 已知法向量构建另外两个正交基
class Onb {
   private:
    Vec3 u;
    Vec3 v;
    Vec3 w;

   public:
    Onb(const Vec3& normal) {
        w = normal;
        Vec3 a = (fabs(w.x) > 0.9) ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
        v = glm::normalize(glm::cross(w, a));
        u = glm::cross(w, v);
    }
    ~Onb() = default;

    // 正交基坐标->世界坐标
    Vec3 local(const Vec3& v) const {
        return v.x * this->u + v.y * this->v + v.z * this->w;
    }
};
}  // namespace SimplePathTracer

#endif