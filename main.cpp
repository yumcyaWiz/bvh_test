#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

#include "vec3.h"
#include "ray.h"
#include "hit.h"
#include "primitive.h"
#include "primitives.h"
#include "camera.h"
#include "aabb.h"
#include "image.h"
#include "rgb.h"


int main() {
    Image img(512, 512);
    Camera cam(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0);
    Primitives prims;
    //prims.add(new Sphere(Vec3(0, 0, 3), 1.0));
    //prims.add(new Sphere(Vec3(1, 0, 3), 1.0));
    prims.add(new Polygon(Vec3(0, 0, 3), "bunny.obj"));
    for(int i = 0; i < 512; i++) {
        for(int j = 0; j < 512; j++) {
            float u = (2.0*i - img.width)/img.width;
            float v = (2.0*j - img.height)/img.height;
            Ray ray = cam.getRay(u, v);
            Hit hit;
            if(prims.intersect_linear(ray, hit))
                img.setPixel(i, j, RGB((hit.hitNormal + 1.0)/2.0));
            else
                img.setPixel(i, j, RGB(0));
        }
    }
    img.ppm_output("output.ppm");
}
/*
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
int main() {
    std::string file = "bunny.obj";
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, file.c_str());
    if(!err.empty())
        std::cerr << err << std::endl;
    if(!ret)
        std::exit(1);
    for(size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];
            for(size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
                std::cout << "Vertex:" << Vec3(vx, vy, vz) << std::endl;
            }
            index_offset += fv;
        }
    }
}
*/
