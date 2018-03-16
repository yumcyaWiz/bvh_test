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
    Primitives prims;
    prims.loadObj(Vec3(0), 100.0f, "buddha.obj");

    Timer timer;
    timer.start();
    prims.constructBVH(4, BVH_PARTITION_TYPE::SAH);
    timer.stop("BVH Construction:");

    AABB objAABB = prims.aabb();

    Vec3 max = objAABB.pMax;
    Vec3 min = objAABB.pMin;
    Vec3 c = (max + min)/2.0f;

    Vec3 ss[101][101];
    Vec3 ds[101][101];
    for(int i = 0; i < 101; i++) {
        for(int j = 0; j < 101; j++) {
            Vec3 s = min + Vec3((max.x - min.x)*(i/100.0f), (max.y - min.y)*(j/100.0f), 0.0f);
            ss[i][j] = s;
            ds[i][j] = normalize(c - s);
        }
    }

    //pretest
    Ray pretest_ray(ss[76][48], ds[76][48]);
    Hit pretest_res;
    bool pretest_hit = prims.intersect(pretest_ray, pretest_res);
    if(!pretest_hit) {
        std::cerr << "Pretest failed" << std::endl;
        std::exit(1);
    }
    else {
        std::cout << "Pretest success!" << std::endl;
        std::cout << "AABB center:" << c << std::endl;
        std::cout << "ray origin:" << pretest_ray.origin << std::endl;
        std::cout << "ray direction:" << pretest_ray.direction << std::endl;
        std::cout << "Pretest hitPos:" << pretest_res.hitPos << std::endl;
        std::cout << "Pretest hitNormal:" << pretest_res.hitNormal << std::endl;
    }

    int nTest = 100;
    double t = 0;
    for(int ccc = 0; ccc < nTest; ccc++) {
        auto start = std::chrono::system_clock::now();
        #pragma omp parallel for schedule(dynamic, 1)
        for(int i = 0; i < 100; i++) {
            for(int j = 0; j < 100; j++) {
                Ray ray(ss[i][j], ds[i][j]);
                Hit res;
                bool hit = prims.intersect(ray, res);
            }
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        t += elapsed;
    }
    //std::cout << "Max Intersection Count:" << max_intersection_count << std::endl;
    std::cout << "Total Ray Count:" << ray_count << std::endl;
    std::cout << "Benchmark:" << t/nTest << "ms" << std::endl;
}
