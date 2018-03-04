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

        AABB() {
            float maxInf = std::numeric_limits<float>::max();
            float minInf = std::numeric_limits<float>::lowest();
            pMin = Vec3(maxInf, maxInf, maxInf);
            pMax = Vec3(minInf, minInf, minInf);
        };
        AABB(const Vec3& p1, const Vec3& p2) : pMin(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)), pMax(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z)) {};

        float surfaceArea() const {
            float dx = pMax.x - pMin.x;
            float dy = pMax.y - pMin.y;
            float dz = pMax.z - pMin.z;
            return 2*(dx*dy + dy*dz + dz*dx);
        };

        bool intersect(const Ray& ray) const {
            float t_min = std::numeric_limits<float>::lowest();
            float t_max = std::numeric_limits<float>::max();
            for(int i = 0; i < 3; i++) {
                float t1 = (pMin[i] - ray.origin[i])/ray.direction[i];
                float t2 = (pMax[i] - ray.origin[i])/ray.direction[i];
                float t_near = std::min(t1, t2);
                float t_far = std::max(t1, t2);
                t_max = std::min(t_max, t_far);
                t_min = std::max(t_min, t_near);
                if(t_min > t_max) return false;
            }
            return true;
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


inline std::ostream& operator<<(std::ostream& stream, const AABB& aabb) {
    stream << "pMin:" << aabb.pMin << " pMax:" << aabb.pMax;
}
#endif
