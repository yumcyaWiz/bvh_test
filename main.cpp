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
    Image* img = new Image(512, 512);
    Camera* cam = new Camera(Vec3(0, 1, -2), Vec3(0, 0, 1), 1.0);

    Primitives* prims = new Primitives();
    
    /*
    for(int i = 0; i < 1000; i++) {
        prims.add(new Sphere(Vec3(i - 500, 0, 3), 1.0));
    }
    */
    
    prims->loadObj(Vec3(0, 0, 0), 10.0f, "bunny.obj");
    //prims->add(new Sphere(Vec3(0, -10000, 0), 10000.0f));
    //prims.add(new Sphere(Vec3(0, -10001.5, 0), 10000));
    Timer timer;
    timer.start();
    prims->constructBVH();
    timer.stop();

    int samples = 100;
    Render render(img, cam, prims, samples);
    timer.start();
    render.render_bvh();
    timer.stop();
    std::cout << "BVH node Intersection Count:" << (float)bvh_intersection_count << std::endl;
    std::cout << "Primitive Intersection Count:" << (float)primitive_intersection_count << std::endl;
    std::cout << "node/primitive intersection:" << (float)primitive_intersection_count/bvh_intersection_count*100 << "%" << std::endl;

    std::cout << "Max Intersection Count:" << max_intersection_count << std::endl;

    render.output();
}
