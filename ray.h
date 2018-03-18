#ifndef RAY_H
#define RAY_H


static int ray_count = 0;


class Ray {
    public:
        Vec3 origin;
        Vec3 direction;
        float tmin = 0.01f;
        float tmax = 100000.0f;

        Ray() {};
        Ray(const Vec3& origin, const Vec3& direction) : origin(origin), direction(direction) {
            ray_count++;
        };

        Vec3 operator()(float t) const {
            return origin + t*direction;
        };
};
#endif
