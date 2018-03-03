#include <iostream>

#include "vec3.h"
#include "ray.h"
#include "primitive.h"
#include "camera.h"
#include "aabb.h"
#include "image.h"
#include "rgb.h"

int main() {
    Image img(512, 512);
    for(int i = 0; i < 512; i++) {
        for(int j = 0; j < 512; j++) {
            img.setPixel(i, j, RGB(i/512.f, j/512.f, 1.0f));
        }
    }
    img.ppm_output("output.ppm");
}
