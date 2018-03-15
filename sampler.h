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
inline Vec3 randomInHemisphere(const Ray& ray, const Vec3& n) {
    Vec3 up = n;
    Vec3 right = normalize(cross(ray.direction, up));
    Vec3 forward = normalize(cross(right, up));
    float x, y, z;
    Vec3 v;
    do {
        x = rnd2();
        y = rnd();
        z = rnd2();
    }
    while(x*x + y*y + z*z > 1.0f);
    return x*right + y*up + z*forward;
}
#endif
