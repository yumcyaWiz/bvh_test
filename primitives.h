#ifndef PRIMITIVES_H
#define PRIMITIVES_H
#include <vector>
#include <memory>
#include "ray.h"
#include "primitive.h"


class BVH_node {
    public:
        int left; //left child index
        int right; //right child index
        std::vector<std::shared_ptr<Primitive>> prims; //primitives(left node only)

        BVH_node() : left(-1), right(-1) {};
};
class BVH {
    public:
        BVH_node* nodes;
        int node_count;

        BVH() {
            nodes = new BVH_node[10000];
            node_count = 0;
        };

        void construct(const std::vector<std::shared_ptr<Primitive>>& prims) {
        };
        bool intersect(const Ray& ray, Hit& res) const {
        };
};


class Primitives {
    public:
        std::vector<std::shared_ptr<Primitive>> prims;

        Primitives() {};

        void add(Primitive* prim) {
            prims.push_back(std::shared_ptr<Primitive>(prim));
        };
        void constructBVH() {
        };
        bool intersect(const Ray& ray, Hit& res) const {
            return false;
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
};
#endif
