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
    Camera* cam = new Camera(Vec3(0, 5, 10), Vec3(0, 0, -1), 1.0);

    Primitives* prims = new Primitives();
    
    /*
    for(int i = 0; i < 1000; i++) {
        prims.add(new Sphere(Vec3(i - 500, 0, 3), 1.0));
    }
    */
    
    prims->loadObj(Vec3(0, 0, 0), "buddha.obj");
    //prims->add(new Sphere(Vec3(0, -10000, 0), 10000.0f));
    //prims.add(new Sphere(Vec3(0, -10001.5, 0), 10000));
    Timer timer;
    timer.start();
    prims->constructBVH();
    timer.stop();

    Render render(img, cam, prims, 50);
    timer.start();
    render.render();
    timer.stop();

    render.output();
}
