#ifndef RAY_H
#define RAY_H
class Ray {
    public:
        Vec3 origin;
        Vec3 direction;

        Ray(const Vec3& origin, const Vec3& direction) : origin(origin), direction(direction) {};

        Vec3 operator()(float t) const {
            return origin + t*direction;
        };
};
#endif
