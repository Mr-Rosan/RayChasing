#pragma once
#ifndef __RAY_HPP__
#define __RAY_HPP__

#include "geometry/vec.hpp"

#include <limits>

#define FLOAT_INF numeric_limits<float>::infinity()
namespace PhotonMap {
using namespace NRenderer;
using namespace std;

struct Photon
{
    Vec3 position;
    Vec3 direction;
    Vec3 power;
    int axis;// this indicates the kdtree dimention

    void setOrigin(const Vec3& v) {
        position = v;
    }

    void setDirection(const Vec3& v) {
        direction = glm::normalize(v);
    }

    void setPow(const Vec3& v) {
        power = v;
    }

    inline
        Vec3 at(float t) const {
        return position + t * direction;
    }

    Photon(const Vec3& origin, const Vec3& direction, const Vec3& power)
        : position(origin)
        , direction(direction)
        , power(power)
    {
        axis = -1;
    }

    Photon()
        : position{}
        , direction{}
        , power{}
    {
        axis = -1;
    }

    float getAxis(int axis) {
        switch (axis) {
            case 0:
            {
                return position.x;
                break;
            }
            case 1:
            {
                return position.y;
                break;
            }
            case 2:
            {
                return position.z;
                break;
            } 
            default:
            {
                return position.x;
                break;
            }
        }
    }

};

class photonMap {
public:
    int p_num;//the number of photon
    int p_max;
    Photon* p_list;//the array of photon
    void store(Photon pn);
    Vec3 box_min, box_max;
    photonMap(int maxnum) :p_max(maxnum)
    {
        p_list = new Photon[maxnum];
        p_num = 0;
        box_min = Vec3(1000000.0f, 1000000.0f, 1000000.0f);
        box_max = Vec3(-1000000.0f, -1000000.0f, -1000000.0f);
    };
    photonMap()
    {
        p_max = 5;
        p_list = new Photon[p_max];
        p_num = 0;
        box_min = Vec3(1000000.0f, 1000000.0f, 1000000.0f);
        box_max = Vec3(-1000000.0f, -1000000.0f, -1000000.0f);
    };// set
    void add_p(Photon* p);// to add photon into my photon list
    int calMedian(int start, int end);
    void MedianSplit(Photon* tempPhoton, int start, int end, int med, int axis);
    float getPhotonPosAxis(int index, int axis);
    void balanceSegment(Photon* tempPhoton, int index, int start, int end);
    ~photonMap() { delete[]p_list; };
    void balance();
};

// make sure direction is a unit vector
struct Ray {
    Vec3 origin;
    // keep it as a unit vector
    Vec3 direction;

    void setOrigin(const Vec3& v) { origin = v; }

    void setDirection(const Vec3& v) { direction = glm::normalize(v); }

    inline Vec3 at(float t) const { return origin + t * direction; }

    Ray(const Vec3& origin, const Vec3& direction)
        : origin(origin), direction(direction) {}

    Ray() : origin{}, direction{} {}
};
}  // namespace PathTracer

#endif