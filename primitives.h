#ifndef PRIMITIVES_H
#define PRIMITIVES_H
#include <vector>
#include <memory>
#include <limits>
#include <algorithm>
#include "ray.h"
#include "primitive.h"
#include "sampler.h"


class BVH_node {
    public:
        BVH_node* left;
        BVH_node* right;
        AABB bbox;
        std::shared_ptr<Primitive> prim;

        BVH_node(std::shared_ptr<Primitive> _prim) {
            left = right = nullptr;
            prim = _prim;
        };
        BVH_node(std::vector<std::shared_ptr<Primitive>>& prims) {
            if(prims.size() == 0) {
                std::cerr << "prims is empty" << std::endl;
                std::exit(1);
            }
            
            int axis = (int)(3*rnd());
            if(axis == 0) {
                std::sort(prims.begin(), prims.end(), [](std::shared_ptr<Primitive> x, std::shared_ptr<Primitive> y) {
                        return x->aabb().pMin.x < y->aabb().pMin.x;
                        });
            }
            else if(axis == 1) {
                std::sort(prims.begin(), prims.end(), [](std::shared_ptr<Primitive> x, std::shared_ptr<Primitive> y) {
                        return x->aabb().pMin.y < y->aabb().pMin.y;
                        });
            }
            else if(axis == 2) {
                std::sort(prims.begin(), prims.end(), [](std::shared_ptr<Primitive> x, std::shared_ptr<Primitive> y) {
                        return x->aabb().pMin.z < y->aabb().pMin.z;
                        });
            }

            if(prims.size() == 1) {
                left = right = nullptr;
                prim = prims[0];
                bbox = prim->aabb();
            }
            else {
                std::size_t const half_size = prims.size()/2;
                std::vector<std::shared_ptr<Primitive>> left_prims(prims.begin(), prims.begin() + half_size);
                std::vector<std::shared_ptr<Primitive>> right_prims(prims.begin() + half_size, prims.end());
                left = new BVH_node(left_prims);
                right = new BVH_node(right_prims);
                AABB bbox_left = left->bbox;
                AABB bbox_right = right->bbox;
                bbox = mergeAABB(bbox_left, bbox_right);
            }
        };

        bool intersect(const Ray& ray, Hit& res) const {
            if(left == nullptr && right == nullptr)
                return prim->intersect(ray, res);

            if(!bbox.intersect(ray))
                return false;

            Hit res_left;
            Hit res_right;
            bool hit_left = left->intersect(ray, res_left);
            bool hit_right = right->intersect(ray, res_right);

            if(hit_left && hit_right) {
                if(res_left.t < res_right.t)
                    res = res_left;
                else
                    res = res_right;
                return true;
            }
            else if(hit_left) {
                res = res_left;
                return true;
            }
            else if(hit_right) {
                res = res_right;
                return true;
            }
            else
                return false;
        };
};


class Primitives {
    public:
        std::vector<std::shared_ptr<Primitive>> prims;
        BVH_node* bvh_root;

        Primitives() {};

        void add(Primitive* prim) {
            prims.push_back(std::shared_ptr<Primitive>(prim));
        };
        void loadObj(const Vec3& center, const std::string& filename) {
            tinyobj::attrib_t attrib;
            std::vector<tinyobj::shape_t> shapes;
            std::vector<tinyobj::material_t> materials;

            std::string err;
            bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str());
            if(!err.empty())
                std::cerr << err << std::endl;
            if(!ret)
                std::exit(1);

            for(size_t s = 0; s < shapes.size(); s++) {
                size_t index_offset = 0;
                for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
                    int fv = shapes[s].mesh.num_face_vertices[f];
                    std::vector<Vec3> vertex;
                    for(size_t v = 0; v < fv; v++) {
                        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                        tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                        tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                        tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
                        vertex.push_back(Vec3((float)vx, (float)vy, (float)vz));
                    }
                    index_offset += fv;
                    prims.push_back(std::shared_ptr<Triangle>(new Triangle(center + vertex[0], center + vertex[1], center + vertex[2])));
                }
            }
        };

        void constructBVH() {
            bvh_root = new BVH_node(prims);
        };
        bool intersect(const Ray& ray, Hit& res) const {
            return bvh_root->intersect(ray, res);
        };
        bool intersect_linear(const Ray& ray, Hit& res) const {
            bool hit = false;
            res.t = ray.tmax;
            for(auto itr = prims.begin(); itr != prims.end(); itr++) {
                Hit res2;
                if((*itr)->intersect(ray, res2)) {
                    hit = true;
                    if(res2.t < res.t)
                        res = res2;
                }
            }
            return hit;
        };
        AABB aabb() const {
            AABB bbox;
            for(auto itr = prims.begin(); itr != prims.end(); itr++) {
                bbox = mergeAABB(bbox, (*itr)->aabb());
            }
            return bbox;
        };
};
#endif
