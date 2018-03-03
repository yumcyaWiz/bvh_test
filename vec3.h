#ifndef VEC3_H
#define VEC3_H
#include <iostream>
#include <cmath>
class Vec3 {
    public:
        float x;
        float y;
        float z;

        Vec3() { x = y = z = 0; };
        Vec3(float x) : x(x), y(x), z(x) {};
        Vec3(float x, float y, float z) : x(x), y(y), z(z) {};

        Vec3 operator+(const Vec3& v) const {
            return Vec3(x + v.x, y + v.y, z + v.z);
        };
        Vec3 operator+(float k) const {
            return Vec3(x + k, y + k, z + k);
        };
        Vec3 operator-(const Vec3& v) const {
            return Vec3(x - v.x, y - v.y, z - v.z);
        };
        Vec3 operator-(float k) const {
            return Vec3(x - k, y - k, z - k);
        };
        Vec3 operator*(const Vec3& v) const {
            return Vec3(x * v.x, y * v.y, z * v.z);
        };
        Vec3 operator*(float k) const {
            return Vec3(x * k, y * k, z * k);
        };
        Vec3 operator/(const Vec3& v) const {
            return Vec3(x / v.x, y / v.y, z / v.z);
        };
        Vec3 operator/(float k) const {
            return Vec3(x / k, y / k, z / k);
        };

        float length() const {
            return std::sqrt(x*x + y*y + z*z);
        };
        float length2() const {
            return x*x + y*y + z*z;
        };
};
inline Vec3 operator+(float k, const Vec3& v) {
    return v + k;
}
inline Vec3 operator-(float k, const Vec3& v) {
    return Vec3(k - v.x, k - v.y, k - v.z);
}
inline Vec3 operator*(float k, const Vec3& v) {
    return v * k;
}
inline Vec3 operator/(float k, const Vec3& v) {
    return Vec3(k / v.x, k / v.y, k / v.z);
}


inline std::ostream& operator<<(std::ostream& stream, const Vec3& v) {
    stream << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}


inline float dot(const Vec3& v1, const Vec3& v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
inline Vec3 cross(const Vec3& v1, const Vec3& v2) {
    return Vec3(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
}
#endif
