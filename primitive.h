#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include <vector>
#include <memory>
#include "vec3.h"
#include "ray.h"
#include "hit.h"
#include "aabb.h"
class Primitive {
    public:
        virtual bool intersect(const Ray& ray, Hit& hit) const = 0;
        virtual AABB aabb() const = 0;
};


class Sphere : Primitive {
    public:
        Vec3 center;
        float radius;

        Sphere(const Vec3& center, float radius) : center(center), radius(radius) {};

        bool intersect(const Ray& ray, Hit& hit) const {
            float b = dot(ray.direction, ray.origin - center);
            float c = (ray.origin - center).length2() - radius*radius;
            float D = b*b - c;
            if(D < 0)
                return false;
            float t1 = -b - std::sqrt(D);
            float t2 = -b + std::sqrt(D);
            if(t1 > ray.tmax || t2 <= ray.tmin)
                return false;
            float tHit = t1;
            if(t1 < ray.tmin) {
                tHit = t2;
                if(tHit > ray.tmax)
                    return false;
            }

            hit.t = tHit;
            hit.hitPos = ray(tHit);
            hit.hitNormal = normalize(hit.hitPos - center);
            return true;
        };
        AABB aabb() const {
            return AABB(center - Vec3(radius), center + Vec3(radius));
        };
};


class Triangle : Primitive {
    public:
        Vec3 p1;
        Vec3 p2;
        Vec3 p3;

        Triangle() {};

        //Moller-Trumbore Algorithm
        bool intersect(const Ray& ray, Hit& hit) const {
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
            if(t <= ray.tmin || t > ray.tmax)
                return false;
            
            hit.t = t;
            hit.hitPos = ray(t);
            return true;
        };
        AABB aabb() const {
            float minX = std::min(std::min(p1.x, p2.x), p3.x);
            float minY = std::min(std::min(p1.y, p2.y), p3.y);
            float minZ = std::min(std::min(p1.z, p2.z), p3.z);
            float maxX = std::max(std::max(p1.x, p2.x), p3.x);
            float maxY = std::max(std::max(p1.y, p2.y), p3.y);
            float maxZ = std::max(std::max(p1.z, p2.z), p3.z);
            return AABB(Vec3(minX, minY, minZ), Vec3(maxX, maxY, maxZ));
        };
};


class Polygon : Primitive {
    public:
        std::vector<std::shared_ptr<Triangle>> triangles;

        bool intersect(const Ray& ray, Hit& hit) const {
            return false;
        };
        AABB aabb() const {
            return AABB();
        };
};
#endif
