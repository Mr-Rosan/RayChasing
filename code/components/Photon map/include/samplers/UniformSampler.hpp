#pragma once
#ifndef __UNIFORM_SAMPLER_HPP__
#define __UNIFORM_SAMPLER_HPP__

#include <ctime>
#include "Sampler1d.hpp"

namespace PhotonMap {
using namespace std;
class UniformSampler : public Sampler1d {
   private:
    default_random_engine e;
    uniform_real_distribution<float> u;

   public:
    UniformSampler() : e((unsigned int)time(0) + insideSeed()), u(0, 1) {}
    float sample1d() override { return u(e); }
};
}  // namespace PathTracer

#endif