#ifndef SAMPLER_H
#define SAMPLER_H
#include <random>
#include "vec3.h"


std::random_device rnd_dev;
std::mt19937 mt(rnd_dev());
std::uniform_real_distribution<float> rand_dist(0, 1);
inline float rnd() {
    return rand_dist(mt);
}
inline float rnd2() {
    return 2.0f*rnd() - 1.0f;
}
inline Vec3 randVec() {
    return Vec3(rnd2(), rnd2(), rnd2());
}

inline Vec3 randomInUnitSphere() {
    Vec3 v = randVec();
    return normalize(v);
};
#endif
