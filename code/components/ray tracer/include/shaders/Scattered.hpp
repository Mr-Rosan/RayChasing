#pragma once
#ifndef __SCATTERED_HPP__
#define __SCATTERED_HPP__

#include "Ray.hpp"

namespace RayTracer
{
    struct Scattered
    {
        Ray ray = {};
        Vec3 attenuation = {};
        Vec3 emitted = {};
        float pdf = {0.f};
    };
    
}

#endif