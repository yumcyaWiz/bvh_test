#ifndef BVH_H
#define BVH_H
class BVH_node {
    public:
        BVH_node* left;
        BVH_node* right;
        Polygon* poly;

        BVH_node() left(nullptr), right(nullptr), poly(nullptr) {};
        BVH_node(const BVH_node* left, const BVH_node* right, const Polygon* poly) : left(left), right(right), poly(poly) {};
};
#endif
