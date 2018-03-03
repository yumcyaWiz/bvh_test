#include <iostream>

#include "vec3.h"
#include "ray.h"
#include "primitive.h"
#include "camera.h"

int main() {
    Vec3 v(1, 1, 1);
    std::cout << v << std::endl;
    std::cout << v[3] << std::endl;
}
