#pragma once
#ifndef __NR_MESH_H__
#define __NR_MESH_H__

#include <vector>
#include "BVH.hpp"
#include "Model.hpp"

namespace NRenderer
{
    using namespace std;
    struct Mesh : public Entity
    {
        vector<Vec3> normals;
        vector<Vec3> points;
        vector<Vec2> uvs;

        vector<Index> normalIndices;
        vector<Index> positionIndices;
        vector<Index> uvIndices;

        BVH* bvh;

        Mesh()
        {
            type = EntityType::MESH;
            material.setIndex(0);
        }

        void calBoundingBox()
        {
            Vec3 minV(FLT_MAX, FLT_MAX, FLT_MAX), maxV(-FLT_MAX, -FLT_MAX, -FLT_MAX);
            for (auto v : points)
            {
                minV = { fmin(v.x, minV.x), fmin(v.y, minV.y), fmin(v.z, minV.z) };
                maxV = { fmax(v.x, maxV.x), fmax(v.y, maxV.y), fmax(v.z, maxV.z) };
            }
            boundingBox = Bounds(minV, maxV);
        }

        void calArea()
        {
            area = 0;
            for (float i = 0; i < positionIndices.size(); i += 3.0)
            {
                Vec3 v1 = points[positionIndices[3.0 * i]];
                Vec3 v2 = points[positionIndices[3.0 * i + 1.0]];
                Vec3 v3 = points[positionIndices[3.0 * i + 2.0]];
                area += glm::length(glm::cross(v2 - v1, v3 - v1)) / 2.0;
            }
        }

        void buildBVH()
        {
            vector<Entity*> triangles;
            hasArea = true;
            area = 0;
            for (float i = 0; i < positionIndices.size(); i += 3.0)
            {
                Vec3 v1 = points[positionIndices[i]];
                Vec3 v2 = points[positionIndices[i + 1.0]];
                Vec3 v3 = points[positionIndices[i + 2.0]];
                Vec3 normal = glm::cross(v2 - v1, v3 - v1);
                area += glm::length(normal) / 2.0;
                normal = glm::normalize(normal);
                triangles.push_back(new Triangle(v1, v2, v3, normal, material.index()));
            }
            bvh = new BVH(triangles);
        }
    };
    SHARE(Mesh);
}  // namespace NRenderer
#endif __NR_MESH_H__