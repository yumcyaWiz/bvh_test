#ifndef UTIL_H
#define UTIL_H
#include <string>
#include <cmath>
#include "rgb.h"


template<typename T>
T clamp(T x, T xmin, T xmax) {
    if(x < xmin)
        return xmin;
    else if(x > xmax)
        return xmax;
    else
        return x;
}


RGB pow(const RGB& col, float n) {
    return RGB(std::pow(col.r, n), std::pow(col.g, n), std::pow(col.b, n));
}


std::string percentage(float x, float max) {
    return std::to_string(x/max*100) + "%";
}
std::string progressbar(float x, float max) {
    const int max_count = 40;
    int cur_count = (int)(x/max * max_count);
    std::string str;
    str += "[";
    for(int i = 0; i < cur_count; i++)
        str += "#";
    for(int i = 0; i < (max_count - cur_count - 1); i++)
        str += " ";
    str += "]";
    return str;
}


float jetcolormap_red(float x) {
    if(x < 0.7) {
        return 4.0*x - 1.5;
    }
    else {
        return -4.0*x + 4.5;
    }
}
float jetcolormap_green(float x) {
    if(x < 0.5) {
        return 4.0*x - 0.5;
    }
    else {
        return -4.0*x + 3.5;
    }
}
float jetcolormap_blue(float x) {
    if(x < 0.3) {
        return 4.0*x + 0.5;
    }
    else {
        return -4.0*x + 2.5;
    }
}
RGB jetcolormap(float x) {
    float r = clamp(jetcolormap_red(x), 0.f, 1.f);
    float g = clamp(jetcolormap_green(x), 0.f, 1.f);
    float b = clamp(jetcolormap_blue(x), 0.f, 1.f);
    return RGB(r, g, b);
}
#endif
