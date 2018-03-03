#ifndef HIT_H
#define HIT_H
#include "vec3.h"
class Hit {
    public:
        float t;
        Vec3 hitPos;
        Vec3 hitNormal;

        Hit(float t, const Vec3& hitPos, const Vec3& hitNormal) : t(t), hitPos(hitPos), hitNormal(hitNormal) {};
};
#endif
