#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <omp.h>
#include <gperftools/profiler.h>

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
#include "render.h"


int main() {
    Image img(512, 512);

    Camera cam(Vec3(0, 5, -10), Vec3(0, 0, 1), 1.0f);

    Primitives prims;
    prims.loadObj(Vec3(0), 1.0f, "dragon.obj");

    Timer timer;
    timer.start();
    prims.constructBVH(4, BVH_PARTITION_TYPE::SAH);
    timer.stop("BVH Construction:");

    Hit res;
    Ray ray = cam.getRay(0, 0);
    ProfilerStart("traverse.prof");
    for(int i = 0; i < 1000000; i++) {
        prims.intersect(ray, res);
    }
    ProfilerStop();

    timer.start();
    ProfilerStart("render.prof");
    for(int i = 0; i < img.width; i++) {
        for(int j = 0; j < img.height; j++) {
            const float u = (2.0*i - img.width)/img.width;
            const float v = -(2.0*j - img.height)/img.height;
            const bool hit = prims.intersect(ray, res);
            if(hit) {
                img.setPixel(i, j, (res.hitNormal + 1.0f)/2.0f);
            }
            else {
                img.setPixel(i, j, RGB(0.0f));
            }
        }
    }
    ProfilerStop();
    timer.stop("Rendering Finished:");
    std::cout << "Max Intersection Count:" << prims.bvh->maximum_intersect_count << std::endl;
    std::cout << "Total Ray Count:" << ray_count << std::endl;
    
    img.gamma_correction();
    img.ppm_output("profile.ppm");
}
