#pragma once
#ifndef __NR_BVH_H__
#define __NR_BVH_H__

#include <algorithm>
#include <vector>
#include "Model.hpp"

namespace NRenderer
{
    using namespace std;
    struct BVHBuildNode
    {
        Bounds bound;
        BVHBuildNode *left, *right;
        Entity* object;
        float area;

        BVHBuildNode()
        {
            bound = Bounds();
            left = nullptr;
            right = nullptr;
            object = nullptr;
        }
    };

    class BVH
    {
    public:
        vector<Entity*> primitives;
        BVHBuildNode* root;

        void sort(vector<Entity*> primitives, int axis)
        {

            if(axis == 0)
            {
                std::sort(primitives.begin(), primitives.end(), [](auto p1, auto p2) {
                    return p1->getBoundingBox().centroid().x < p2->getBoundingBox().centroid().x;
                    });
            }
            else if(axis == 1)
            {
                std::sort(primitives.begin(), primitives.end(), [](auto p1, auto p2) {
                        return p1->getBoundingBox().centroid().y < p2->getBoundingBox().centroid().y;
                    });
            }
            else
            {
                std::sort(primitives.begin(), primitives.end(), [](auto p1, auto p2) {
                        return p1->getBoundingBox().centroid().z < p2->getBoundingBox().centroid().z;
                    });
            }
        }

        BVH(vector<Entity*> objects)
        {
            if (objects.empty())
                return;
            root = recursiveBuild(objects);
            this->primitives = move(objects);
        }

        BVHBuildNode* recursiveBuild(vector<Entity*> objects)
        {
            BVHBuildNode* bnode = new BVHBuildNode();
            Bounds bound;
            for (auto opt : objects)
                bound = Bounds::Union(bound, opt->getBoundingBox());

            auto objSize = objects.size();
            if (objSize == 1)
            {
                bnode->bound = objects[0]->getBoundingBox();
                bnode->object = objects[0];
                bnode->left = nullptr;
                bnode->right = nullptr;
                bnode->area = objects[0]->getArea();
                return bnode;
            }
            else if (objSize == 2)
            {
                bnode->left = recursiveBuild(vector{objects[0]});
                bnode->right = recursiveBuild(vector{objects[1]});
                bnode->bound = Bounds::Union(bnode->left->bound, bnode->right->bound);
                bnode->area = bnode->left->area + bnode->right->area;
                return bnode;
            }
            else
            {
                auto begin = objects.begin();
                auto end = objects.end();
                if (objSize < 12)
                {
                    Bounds cenBound;
                    for (auto p : objects)
                        cenBound = Bounds::Union(cenBound, p->getBoundingBox().centroid());
                    int axis = cenBound.getMaxAxis();
                    sort(objects, axis);

                    auto middle = begin + objSize / 2;
                    bnode->left = recursiveBuild(vector<Entity*>(begin, middle));
                    bnode->right = recursiveBuild(vector<Entity*>(middle, end));
                    bnode->bound = Bounds::Union(bnode->left->bound, bnode->right->bound);
                    bnode->area = bnode->left->area + bnode->right->area;
                }
                else
                {
                    auto middle = begin;
                    int axis = 0, half = 0;
                    float cost = (numeric_limits<float>::max)();
                    for (int testAxis = 0; testAxis < 3; testAxis++)
                    {
                        sort(objects, testAxis);
                        Bounds left, right;
                        for (float i = 0.0 ; i < 6.0 ;i++)
                        {
                            int temp = int(i / 6 * objSize);
                            for (int i = 0; i < temp; i++)
                                left = Bounds::Union(left, objects[i]->getBoundingBox());

                            for (int i = temp; i < objSize; i++)
                                right = Bounds::Union(right, objects[i]->getBoundingBox());

                            float tempCost = (temp * left.calSurfaceArea() + (objSize - temp) * right.calSurfaceArea()) / bound.calSurfaceArea();
                            if (tempCost < cost)
                            {
                                cost = tempCost;
                                axis = testAxis;
                                half = temp;
                            }
                        }
                    }
                    sort(objects, axis);
                    middle = begin + half;
                    bnode->left = recursiveBuild(vector<Entity*>(begin, middle));
                    bnode->right = recursiveBuild(vector<Entity*>(middle, end));
                    bnode->bound = Bounds::Union(bnode->left->bound, bnode->right->bound);
                    bnode->area = bnode->left->area + bnode->right->area;
                }
            }
            return bnode;
        }
    };
}  // namespace NRenderer
#endif __NR_BVH_H__