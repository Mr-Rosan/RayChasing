#pragma once
#ifndef __NR_MODEL_HPP__
#define __NR_MODEL_HPP__

#include <string>
#include <vector>

#include "geometry/vec.hpp"

#include "Material.hpp"
#include "common/macros.hpp"
namespace NRenderer {
using namespace std;


inline Vec3 minVec(const Vec3& v1, const Vec3& v2)
{
    return { fmin(v1.x, v2.x), fmin(v1.y, v2.y), fmin(v1.z, v2.z) };
}

inline Vec3 maxVec(const Vec3& v1, const Vec3& v2)
{
    return { fmax(v1.x, v2.x), fmax(v1.y, v2.y), fmax(v1.z, v2.z) };
}

class Bounds
{
private:
    Vec3 pMin;
    Vec3 pMax;

public:
    Vec3 getMinPoint()const { return pMin; }
    Vec3 getMaxPoint()const { return pMax; }

    Bounds()
    {
        pMax = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
        pMin = { FLT_MAX, FLT_MAX, FLT_MAX };
    }

    Bounds(const Vec3& p1, const Vec3& p2)
    {
        pMin = { fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z) };
        pMax = { fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z) };
    }

    int getMaxAxis() const
    {
        Vec3 side = pMax - pMin;
        if (side.x > side.y && side.x > side.z)
            return 0;
        else if (side.y > side.z)
            return 1;
        else
            return 2;
    }

    float calSurfaceArea() const
    {
        Vec3 side = pMax - pMin;
        return 2 * (side.x * side.y + side.x * side.z + side.y * side.z);
    }

    Vec3 centroid() const
    {
        return (pMin + pMax) * 0.5f;
    }

    Bounds getIntersection(const Bounds& b)
    {
        return Bounds(maxVec(pMin, b.pMin), minVec(pMax, b.pMax));
    }

    Vec3 getOffset(const Vec3& p)
    {
        Vec3 off = p - pMin;
        if (pMax.x > pMin.x)
            off.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            off.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            off.z /= pMax.z - pMin.z;
    }

    static bool overlaps(const Bounds& b1, const Bounds& b2)
    {
#define TEST(x) \
        (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x)
        return TEST(x) && TEST(y) && TEST(z);
    }

    static bool inside(const Vec3& v, const Bounds& b)
    {
#define TEST1(x) (v.x >= b.pMin.x) && (v.x <= b.pMax.x)
        return TEST1(x) && TEST1(y) && TEST1(z);
    }

    static inline Bounds Union(const Bounds& b1, const Bounds& b2)
    {
        Bounds ret;
        ret.pMin = minVec(b1.pMin, b2.pMin);
        ret.pMax = maxVec(b1.pMax, b2.pMax);
        return ret;
    }

    static inline Bounds Union(const Bounds& b, const Vec3& v)
    {
        Bounds ret;
        ret.pMin = minVec(b.pMin, v);
        ret.pMax = maxVec(b.pMax, v);
        return ret;
    }
};

struct Entity
{
    enum class EntityType { ENTITY, SPHERE, TRIANGLE, PLANE, MESH };
    EntityType type = EntityType::ENTITY;
    Handle material;
    Bounds boundingBox;
    float area;
    bool hasBound = false;
    bool hasArea = false;
    virtual void calBoundingBox() {}
    virtual void calArea() {}
    Bounds getBoundingBox() {
        if (!hasBound) {
            hasBound = true;
            calBoundingBox();
        }
        return boundingBox;
    }
    float getArea() {
        if (!hasArea) {
            hasArea = true;
            calArea();
        }
        return area;
    }
};
SHARE(Entity);

struct Sphere : public Entity
{
    // 球的方向(可以当作极轴的方向)
    Vec3 direction = {0, 0, 1};
    Vec3 position = {0, 0, 0};
    float radius = {0};
    Sphere() { type = EntityType::SPHERE; }
    void calBoundingBox() {
        Vec3 r = {radius, radius, radius};
        boundingBox = Bounds(position - r, position + r);
    }
    void calArea() {
        constexpr float PI = 3.1415926535898f;
        area = 4 * PI * radius * radius;
    }
};
SHARE(Sphere);

struct Triangle : public Entity
{
    union
    {
        struct
        {
            Vec3 v1;
            Vec3 v2;
            Vec3 v3;
        };
        Vec3 v[3];
    };
    Vec3 normal;

    Triangle() : v1(), v2(), v3(), normal(0, 0, 1)
    {
        type = EntityType::TRIANGLE;
    }

    Triangle(Vec3 v1, Vec3 v2, Vec3 v3, Vec3 normal, int material) : v1(v1), v2(v2), v3(v3), normal(normal)
    {
        type = EntityType::TRIANGLE;
        this->material.setIndex(material);
    };
    void calBoundingBox() { boundingBox = Bounds::Union(Bounds(v1, v2), v3); }
    void calArea() { area = glm::length(glm::cross(v2 - v1, v3 - v1)) / 2; }
};
SHARE(Triangle);

struct Plane : public Entity
{
    Vec3 normal = {0, 0, 1};
    Vec3 position = {};
    // u, v 是两条边
    Vec3 u = {};
    Vec3 v = {};
    Plane() { type = EntityType::PLANE; }
    void calBoundingBox()
    {
        Vec3 v2 = position + u;
        Vec3 v3 = position + v;
        Vec3 v4 = position + u + v;
        boundingBox = Bounds::Union(Bounds(position, v2), Bounds(v3, v4));
    }
    void calArea() { area = glm::length(glm::cross(u, v)); }
};
SHARE(Plane);

struct Node
{
    enum class Type { SPHERE = 0x0, TRIANGLE = 0X1, PLANE = 0X2, MESH = 0X3 };
    Type type = Type::SPHERE;
    Index entity;
    Index model;
};
SHARE(Node);

struct Model
{
    vector<Index> nodes;
    // 在世界坐标的位置
    Vec3 translation = {0, 0, 0};
    // 缩放
    Vec3 scale = {1, 1, 1};
};
SHARE(Model);
}  // namespace NRenderer

#endif