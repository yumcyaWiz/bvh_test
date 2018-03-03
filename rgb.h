#ifndef RGB_H
#define RGB_H
#include <iostream>
class RGB {
    public:
        float r;
        float g;
        float b;

        RGB() { r = g = b = 0; };
        RGB(float r, float g, float b) : r(r), g(g), b(b) {};

        RGB operator+(const RGB& col) const {
            return RGB(r + col.r, g + col.g, b + col.b);
        };
        RGB operator+(float k) const {
            return RGB(r + k, g + k, b + k);
        };
        RGB operator-(const RGB& col) const {
            return RGB(r - col.r, g - col.g, b - col.b);
        };
        RGB operator-(float k) const {
            return RGB(r - k, g - k, b - k);
        };
        RGB operator*(const RGB& col) const {
            return RGB(r * col.r, g * col.g, b * col.b);
        };
        RGB operator*(float k) const {
            return RGB(r * k, g * k, b * k);
        };
        RGB operator/(const RGB& col) const {
            return RGB(r / col.r, g / col.g, b / col.b);
        };
        RGB operator/(float k) const {
            return RGB(r / k, g / k, b / k);
        };
};
inline RGB operator+(float k, const RGB& col) {
    return col + k;
}
inline RGB operator-(float k, const RGB& col) {
    return RGB(k - col.r, k - col.g, k - col.b);
}
inline RGB operator*(float k, const RGB& col) {
    return RGB(k * col.r, k * col.g, k * col.b);
}
inline RGB operator/(float k, const RGB& col) {
    return RGB(k / col.r, k / col.g, k / col.b);
}


inline std::ostream& operator<<(std::ostream& stream, const RGB& col) {
    stream << "(" << col.r << ", " << col.g << ", " << col.b << ")";
}
#endif
