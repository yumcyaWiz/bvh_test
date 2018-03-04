#ifndef PRIMITIVES_H
#define PRIMITIVES_H
#include <vector>
#include <memory>
#include "ray.h"
#include "primitive.h"


class BVH_node {
    public:
        BVH_node* left;
        BVH_node* right;
        std::vector<std::shared_ptr<Primitive>> prims;

        BVH_node() : left(nullptr), right(nullptr) {};
        BVH_node(BVH_node* left, BVH_node* right, std::shared_ptr<Primitive> prim) : left(left), right(right), prims(prims) {};
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
        bool intersect(const Ray& ray) const {
        };
};
#endif
