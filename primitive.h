#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include "vec3.h"
#include "ray.h"
#include "hit.h"
class Primitive {
    public:
        virtual bool intersect(const Ray& ray, const Hit& hit) const = 0;
};


class Sphere : Primitive {
    public:
        Vec3 center;
        float radius;

        Sphere() {};

        bool intersect(const Ray& ray, const Hit& hit) const {
            return false;
        };
};


class Triangle : Primitive {
    public:
        Vec3 p1;
        Vec3 p2;
        Vec3 p3;

        Triangle() {};

        bool intersect(const Ray& ray, const Hit& hit) const {
            return false;
        };
};


class AABB : Primitive {
    public:
        Vec3 pMin;
        Vec3 pMax;

        AABB() {};

        bool intersect(const Ray& ray, const Hit& hit) const {
            return false;
        };
};
#endif
