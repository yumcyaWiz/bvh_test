#ifndef AABB_H
#define AABB_H
#include <iostream>
#include <limits>
#include <algorithm>
#include "vec3.h"
#include "ray.h"
class AABB {
    public:
        Vec3 pMin;
        Vec3 pMax;
        Vec3 center;

        AABB() {
            float maxInf = std::numeric_limits<float>::max();
            float minInf = std::numeric_limits<float>::lowest();
            pMin = Vec3(maxInf, maxInf, maxInf);
            pMax = Vec3(minInf, minInf, minInf);
            center = Vec3(0);
        };
        AABB(const Vec3& p1, const Vec3& p2) : pMin(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)), pMax(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z)) {
            center = (pMin + pMax)/2;
        };

        Vec3 operator[](int i) const {
            if(i == 0)
                return pMin;
            else
                return pMax;
        };

        float surfaceArea() const {
            float dx = pMax.x - pMin.x;
            float dy = pMax.y - pMin.y;
            float dz = pMax.z - pMin.z;
            return 2*(dx*dy + dy*dz + dz*dx);
        };

        bool intersect(const Ray& ray) const {
            float t0 = ray.tmin;
            float t1 = ray.tmax;
            for(int i = 0; i < 3; i++) {
                float tNear = (pMin[i] - ray.origin[i])/ray.direction[i];
                float tFar = (pMax[i] - ray.origin[i])/ray.direction[i];
                
                if(tNear > tFar) std::swap(tNear, tFar);

                t0 = tNear > t0 ? tNear : t0;
                t1 = tFar < t1 ? tFar : t1;
                if(t0 > t1) return false;
            }
            return true;
        };


        bool pointOnEdge(const Vec3& p, float thre_factor) const {
            float xthre = (pMax.x - pMin.x)*thre_factor;
            float ythre = (pMax.y - pMin.y)*thre_factor;
            float zthre = (pMax.z - pMin.z)*thre_factor;
            xthre = thre_factor;
            ythre = thre_factor;
            zthre = thre_factor;
            return (
                    (std::abs(p.x - pMax.x) < xthre && std::abs(p.z - pMin.z) < zthre) ||
                    (std::abs(p.x - pMax.x) < xthre && std::abs(p.z - pMax.z) < zthre) ||
                    (std::abs(p.x - pMin.x) < zthre && std::abs(p.z - pMin.z) < zthre) ||
                    (std::abs(p.x - pMin.x) < zthre && std::abs(p.z - pMax.z) < zthre) ||
                    (std::abs(p.y - pMin.y) < ythre && std::abs(p.z - pMin.z) < zthre) ||
                    (std::abs(p.y - pMin.y) < ythre && std::abs(p.z - pMax.z) < zthre) ||
                    (std::abs(p.y - pMax.y) < ythre && std::abs(p.z - pMin.z) < zthre) ||
                    (std::abs(p.y - pMax.y) < ythre && std::abs(p.z - pMax.z) < zthre) ||
                    (std::abs(p.y - pMin.y) < ythre && std::abs(p.x - pMin.x) < xthre) ||
                    (std::abs(p.y - pMin.y) < ythre && std::abs(p.x - pMax.x) < xthre) ||
                    (std::abs(p.y - pMax.y) < ythre && std::abs(p.x - pMin.x) < xthre) ||
                    (std::abs(p.y - pMax.y) < ythre && std::abs(p.x - pMax.x) < xthre));
        };
        bool intersect_visualize(const Ray& ray, bool &edge) const {
            float t0 = ray.tmin;
            float t1 = ray.tmax;
            for(int i = 0; i < 3; i++) {
                float tNear = (pMin[i] - ray.origin[i])/ray.direction[i];
                float tFar = (pMax[i] - ray.origin[i])/ray.direction[i];
                
                if(tNear > tFar) std::swap(tNear, tFar);

                t0 = tNear > t0 ? tNear : t0;
                t1 = tFar < t1 ? tFar : t1;
                if(t0 > t1) return false;
            }
            Vec3 hitPos_near = ray(t0);
            Vec3 hitPos_far = ray(t1);
            constexpr float thre = 0.03f;
            edge = (pointOnEdge(hitPos_near, thre) || pointOnEdge(hitPos_far, thre));

            /*
            Vec3 p1 = pMin;
            Vec3 p2 = Vec3(pMax.x, pMin.y, pMin.z);
            Vec3 p3 = Vec3(pMax.x, pMin.y, pMax.z);
            Vec3 p4 = Vec3(pMin.x, pMax.y, pMin.z);
            Vec3 p5 = Vec3(pMin.x, pMax.y, pMin.z);
            Vec3 p6 = Vec3(pMax.x, pMin.y, pMax.z);
            Vec3 p7 = pMax;
            Vec3 p8 = Vec3(pMin.x, pMax.y, pMax.z);

            constexpr float thre = 0.1f;
            Vec3 dir1, dir2;
            dir1 = normalize(p1 - hitPos_near);
            dir2 = normalize(p7 - hitPos_near);
            if(std::abs(dot(dir1, dir2)) < thre) 
                edge = true;
            dir1 = normalize(p2 - hitPos_near);
            dir2 = normalize(p8 - hitPos_near);
            if(std::abs(dot(dir1, dir2)) < thre)
                edge = true;
            dir1 = normalize(p3 - hitPos_near);
            dir2 = normalize(p5 - hitPos_near);
            if(std::abs(dot(dir1, dir2)) < thre)
                edge = true;

            dir1 = normalize(p1 - hitPos_far);
            dir2 = normalize(p7 - hitPos_far);
            if(std::abs(dot(dir1, dir2)) < thre)
                edge = true;
            dir1 = normalize(p2 - hitPos_far);
            dir2 = normalize(p8 - hitPos_far);
            if(std::abs(dot(dir1, dir2)) < thre)
                edge = true;
            dir1 = normalize(p3 - hitPos_far);
            dir2 = normalize(p5 - hitPos_far);
            if(std::abs(dot(dir1, dir2)) < thre)
                edge = true;
            */

            return true;
        };
        bool intersect2(const Ray& ray, const Vec3& invdir, const int dirIsNeg[3]) const {
            const AABB& bounds = *this;

            float tMin = (bounds[dirIsNeg[0]].x - ray.origin.x) * invdir.x;
            float tMax = (bounds[1 - dirIsNeg[0]].x - ray.origin.x) * invdir.x;
            const float tyMin = (bounds[dirIsNeg[1]].y - ray.origin.y) * invdir.y;
            const float tyMax = (bounds[1 - dirIsNeg[1]].y - ray.origin.y) * invdir.y;
            if(tMin > tyMax || tyMin > tMax)
                return false;
            if(tyMin > tMin) tMin = tyMin;
            if(tyMax < tMax) tMax = tyMax;

            const float tzMin = (bounds[dirIsNeg[2]].z - ray.origin.z) * invdir.z;
            const float tzMax = (bounds[1 - dirIsNeg[2]].z - ray.origin.z) * invdir.z;
            if(tMin > tzMax || tzMin > tMax)
                return false;
            if(tzMin > tMin) tMin = tzMin;
            if(tzMax < tMax) tMax = tzMax;

            return (tMin < ray.tmax) && (tMax > ray.tmin);
        }

        Vec3 offset(const Vec3& p) const {
            Vec3 o = p - pMin;
            o.x /= (pMax.x - pMin.x);
            o.y /= (pMax.y - pMin.y);
            o.z /= (pMax.z - pMin.z);
            return o;
        };
};


inline AABB mergeAABB(const AABB& b1, const AABB& b2) {
    float minX = std::min(b1.pMin.x, b2.pMin.x);
    float minY = std::min(b1.pMin.y, b2.pMin.y);
    float minZ = std::min(b1.pMin.z, b2.pMin.z);
    float maxX = std::max(b1.pMax.x, b2.pMax.x);
    float maxY = std::max(b1.pMax.y, b2.pMax.y);
    float maxZ = std::max(b1.pMax.z, b2.pMax.z);
    return AABB(Vec3(minX, minY, minZ), Vec3(maxX, maxY, maxZ));
}
inline AABB mergeAABB(const AABB& b1, const Vec3& p) {
    float minX = std::min(b1.pMin.x, p.x);
    float minY = std::min(b1.pMin.y, p.y);
    float minZ = std::min(b1.pMin.z, p.z);
    float maxX = std::max(b1.pMax.x, p.x);
    float maxY = std::max(b1.pMax.y, p.y);
    float maxZ = std::max(b1.pMax.z, p.z);
    return AABB(Vec3(minX, minY, minZ), Vec3(maxX, maxY, maxZ));
}


inline int maximumExtent(const AABB& b) {
    float dx = b.pMax.x - b.pMin.x;
    float dy = b.pMax.y - b.pMin.y;
    float dz = b.pMax.z - b.pMin.z;
    float max = std::max(dx, std::max(dy, dz));
    if(max == dx)
        return 0;
    else if(max == dy)
        return 1;
    else
        return 2;
}


inline std::ostream& operator<<(std::ostream& stream, const AABB& aabb) {
    stream << "pMin:" << aabb.pMin << " pMax:" << aabb.pMax;
    return stream;
}
#endif
