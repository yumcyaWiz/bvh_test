#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <omp.h>

#include "vec3.h"
#include "ray.h"
#include "hit.h"
#include "primitive.h"
#include "primitives.h"
#include "camera.h"
#include "aabb.h"
#include "image.h"
#include "rgb.h"
#include "util.h"


int main() {
    Image img(512, 512);
    Camera cam(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0);

    Primitives prims;
    for(int i = 0; i < 3; i++) {
        prims.add(new Sphere(Vec3(i - 1.5, 0, 3), 1.0));
    }
    //prims.loadObj(Vec3(0, -0.1, 0.2), "bunny.obj");
    prims.constructBVH();

    //#pragma omp parallel for schedule(dynamic, 1)
    for(int i = 0; i < 512; i++) {
        for(int j = 0; j < 512; j++) {
            float u = (2.0*i - img.width)/img.width;
            float v = (2.0*j - img.height)/img.height;
            Ray ray = cam.getRay(u, v);
            Hit hit;
            if(prims.intersect(ray, hit)) {
                img.setPixel(i, j, RGB((hit.hitNormal + 1.0)/2.0));
            }
            else {
                img.setPixel(i, j, RGB(0));
            }
        }
        if(omp_get_thread_num() == 0)
            std::cout << progressbar(i, 512) << " " << percentage(i, 512) << "\r" << std::flush;
    }
    img.ppm_output("output.ppm");
}
