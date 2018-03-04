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

        Sphere(const Vec3& center, float radius) : center(center), radius(radius) {};

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

        //Moller-Trumbore Algorithm
        bool intersect(const Ray& ray, const Hit& hit) const {
            const float eps = 0.000001;
            Vec3 edge1 = p2 - p1;
            Vec3 edge2 = p3 - p1;
            Vec3 h = cross(ray.direction, edge2);
            float a = dot(edge1, h);
            if(a > -eps && a < eps)
                return false;
            float f = 1/a;
            Vec3 s = ray.origin - p1;
            float u = f*dot(s, h);
            if(u < 0.0 || u > 1.0)
                return false;
            Vec3 q = cross(s, edge1);
            float v = f*dot(ray.direction, q);
            if(v < 0.0 || u + v > 1.0)
                return false;
            float t = f*dot(edge2, q);
            
            hit.t = t;
            hit.hitPos = ray(t);
            return true;
        };
};
#endif
