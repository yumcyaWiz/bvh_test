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
#include "render.h"


int main() {
    Image img(512, 512);

    Camera cam(Vec3(0, 5, -10), Vec3(0, 0, 1), 1.0f);

    BVH prims(4, BVH_PARTITION_TYPE::SAH);
    prims.loadObj(Vec3(0), 1.0f, "dragon.obj");

    Timer timer;
    timer.start();
    prims.constructBVH();
    timer.stop("BVH Construction:");

    int nTest = 100;
    double t = 0;
    for(int ccc = 0; ccc < nTest; ccc++) {
        const auto start = std::chrono::system_clock::now();
        for(int i = 0; i < img.width; i++) {
        #pragma omp parallel for schedule(dynamic, 1)
            for(int j = 0; j < img.height; j++) {
                const float u = (2.0*i - img.width)/img.width;
                const float v = -(2.0*j - img.height)/img.height;
                Ray ray = cam.getRay(u, v);
                Hit res;
                const bool hit = prims.intersect(ray, res);
                if(hit) {
                    img.setPixel(i, j, (res.hitNormal + 1.0f)/2.0f);
                }
                else {
                    img.setPixel(i, j, RGB(0.0f));
                }
            }
        }
        const auto end = std::chrono::system_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        t += elapsed;
    }
    std::cout << "Max Intersection Count:" << prims.maximum_intersect_count << std::endl;
    std::cout << "Total Ray Count:" << ray_count << std::endl;
    std::cout << "Benchmark:" << t/nTest << "ms" << std::endl;
    
    img.gamma_correction();
    img.ppm_output("benchmark.ppm");
}
