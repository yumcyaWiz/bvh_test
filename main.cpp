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
#include "timer.h"


int main() {
    Image img(512, 512);
    Camera cam(Vec3(0, 100, 0), Vec3(1, 0, 0), 1.0);

    Primitives prims;
    
    /*
    for(int i = 0; i < 1000; i++) {
        prims.add(new Sphere(Vec3(i - 500, 0, 3), 1.0));
    }
    */
    
    prims.loadObj(Vec3(0, 0, 0), "sponza_simple.obj");
    //prims.add(new Sphere(Vec3(0, -10001.5, 0), 10000));
    Timer timer;
    timer.start();
    prims.constructBVH();
    timer.stop();
    

    timer.start();
    #pragma omp parallel for schedule(dynamic, 1)
    for(int i = 0; i < 512; i++) {
        for(int j = 0; j < 512; j++) {
            float u = (2.0*i - img.width)/img.width;
            float v = (2.0*j - img.height)/img.height;
            Ray ray = cam.getRay(u, v);
            Hit hit;
            if(prims.intersect(ray, hit)) {
                RGB col = RGB((-hit.hitNormal + 1.0)/2.0);
                img.setPixel(i, j, col);
            }
            else {
                img.setPixel(i, j, RGB(0));
            }
        }
        if(omp_get_thread_num() == 0)
            std::cout << progressbar(i, 512) << " " << percentage(i, 512) << "\r" << std::flush;
    }
    timer.stop();
    img.ppm_output("output.ppm");
}
