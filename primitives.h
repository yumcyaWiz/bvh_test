#ifndef PRIMITIVES_H
#define PRIMITIVES_H
#include <vector>
#include <memory>
#include <limits>
#include <algorithm>
#include "ray.h"
#include "primitive.h"
#include "sampler.h"


static int node_count = 0;
static int leaf_count = 0;

static int xsplit_count = 0;
static int ysplit_count = 0;
static int zsplit_count = 0;

static int bvh_intersection_count = 0;
static int primitive_intersection_count = 0;



enum class BVH_PARTITION_TYPE {
    EQSIZE,
    CENTER,
    SAH
};


class BVH {
    private:
        struct BVH_node {
            public:
                std::shared_ptr<BVH_node> left; //left child pointer
                std::shared_ptr<BVH_node> right; //right child pointer
                AABB bbox; //node bounding box
                std::vector<std::shared_ptr<Primitive>> prim; //node primitives(leaf node only)
                int splitAxis; //splitting axis

                AABB computeBounds(const std::vector<std::shared_ptr<Primitive>>& prims) const {
                    AABB ret;
                    for(auto itr = prims.begin(); itr != prims.end(); itr++) {
                        ret = mergeAABB(ret, (*itr)->aabb());
                    }
                    return ret;
                };
                bool prim_intersect(const std::vector<std::shared_ptr<Primitive>>& prims, Ray& ray, Hit& res) const {
                    bool hit = false;
                    res.t = ray.tmax;
                    Hit prim_res;
                    for(auto itr = prims.begin(); itr != prims.end(); itr++) {
                        bool prim_hit = (*itr)->intersect(ray, prim_res);
                        if(prim_hit) {
                            hit = true;
                            if(prim_res.t < res.t) {
                                res = prim_res;
                            }
                        }
                    }
                    return hit;
                }

                BVH_node(std::shared_ptr<Primitive> _prim) {
                    left = right = nullptr;
                    prim.push_back(_prim);
                };
                BVH_node(std::vector<std::shared_ptr<Primitive>>& prims, BVH_PARTITION_TYPE partition_type) {
                    node_count++;

                    if(prims.size() == 0) {
                        std::cerr << "prims is empty" << std::endl;
                        std::exit(1);
                    }
                    else if(prims.size() <= 4) {
                        leaf_count++;
                        left = right = nullptr;
                        for(auto itr = prims.begin(); itr != prims.end(); itr++) {
                            prim.push_back((*itr));
                        }
                        bbox = computeBounds(prims);
                        return;
                    }

                    //compute primitive centroid bounds
                    AABB centroidBounds;
                    for(auto itr = prims.begin(); itr != prims.end(); itr++) {
                        centroidBounds = mergeAABB(centroidBounds, (*itr)->center);
                    }
                    
                    //choose maximum extent axis
                    int axis = maximumExtent(centroidBounds);
                    if(axis == 0)
                        xsplit_count++;
                    else if(axis == 1)
                        ysplit_count++;
                    else
                        zsplit_count++;
                    splitAxis = axis;

                    //if centroidBounds are degenerate make this node as leaf
                    if(centroidBounds.pMin[axis] == centroidBounds.pMax[axis]) {
                        leaf_count++;
                        left = right = nullptr;
                        for(auto itr = prims.begin(); itr != prims.end(); itr++)
                            prim.push_back((*itr));
                        bbox = computeBounds(prim);
                        return;
                    }

                    if(partition_type == BVH_PARTITION_TYPE::EQSIZE) {
                        std::size_t half_size = prims.size()/2;
                        std::nth_element(prims.begin(), prims.begin() + half_size, prims.end(), [axis](std::shared_ptr<Primitive> x, std::shared_ptr<Primitive> y) {
                                return x->aabb().center[axis] < y->aabb().center[axis];
                                });

                        std::vector<std::shared_ptr<Primitive>> left_prims(prims.begin(), prims.begin() + half_size);
                        std::vector<std::shared_ptr<Primitive>> right_prims(prims.begin() + half_size, prims.end());

                        left = std::shared_ptr<BVH_node>(new BVH_node(left_prims, partition_type));
                        right = std::shared_ptr<BVH_node>(new BVH_node(right_prims, partition_type));
                        bbox = mergeAABB(left->bbox, right->bbox);
                    }
                    else if(partition_type == BVH_PARTITION_TYPE::CENTER) {
                        //compute mid point
                        float pmid = 0.5f*centroidBounds.pMin[axis] + 0.5f*centroidBounds.pMax[axis];
                        //partition primitives by mid point
                        auto midPtr = std::partition(prims.begin(), prims.end(), [axis, pmid](std::shared_ptr<Primitive> x) {
                                return x->center[axis] < pmid;
                                });

                        std::vector<std::shared_ptr<Primitive>> left_prims(prims.begin(), midPtr);
                        std::vector<std::shared_ptr<Primitive>> right_prims(midPtr, prims.end());

                        left = std::shared_ptr<BVH_node>(new BVH_node(left_prims, partition_type));
                        right = std::shared_ptr<BVH_node>(new BVH_node(right_prims, partition_type));
                        bbox = mergeAABB(left->bbox, right->bbox);
                    }
                    else {
                        constexpr int nBuckets = 12;
                        struct BucketInfo {
                            int count = 0;
                            AABB bounds;
                        };
                        BucketInfo buckets[nBuckets];

                        AABB bounds;
                        for(auto itr = prims.begin(); itr != prims.end(); itr++) {
                            AABB prim_aabb = (*itr)->aabb();
                            bounds = mergeAABB(bounds, prim_aabb);
                            int b = nBuckets * centroidBounds.offset((*itr)->center)[axis];
                            if(b == nBuckets) b = nBuckets - 1;
                            buckets[b].count++;
                            buckets[b].bounds = mergeAABB(buckets[b].bounds, prim_aabb);
                        }

                        float cost[nBuckets - 1];
                        for(int i = 0; i < nBuckets - 1; i++) {
                            AABB b0, b1;
                            int count0 = 0, count1 = 0;
                            for(int j = 0; j <= i; j++) {
                                b0 = mergeAABB(b0, buckets[j].bounds);
                                count0 += buckets[j].count;
                            }
                            for(int j = i+1; j < nBuckets; j++) {
                                b1 = mergeAABB(b1, buckets[j].bounds);
                                count1 += buckets[j].count;
                            }
                            cost[i] = 0.125f + (count0 * b0.surfaceArea() + count1 * b1.surfaceArea()) / bounds.surfaceArea();
                        }

                        float minCost = cost[0];
                        int minCostSplitBucket = 0;
                        for(int i = 1; i < nBuckets - 1; i++) {
                            if(cost[i] < minCost) {
                                minCost = cost[i];
                                minCostSplitBucket = i;
                            }
                        }

                        float leafCost = prims.size();
                        if(minCost < leafCost) {
                            auto midPtr = std::partition(prims.begin(), prims.end(), [=](std::shared_ptr<Primitive> x) {
                                    int b = nBuckets * centroidBounds.offset(x->center)[axis];
                                    if(b == nBuckets) b = nBuckets - 1;
                                    return b <= minCostSplitBucket;
                                    });

                            std::vector<std::shared_ptr<Primitive>> left_prims(prims.begin(), midPtr);
                            std::vector<std::shared_ptr<Primitive>> right_prims(midPtr, prims.end());

                            left = std::shared_ptr<BVH_node>(new BVH_node(left_prims, partition_type));
                            right = std::shared_ptr<BVH_node>(new BVH_node(right_prims, partition_type));
                            bbox = mergeAABB(left->bbox, right->bbox);
                        }
                        else {
                            leaf_count++;
                            left = right = nullptr;
                            for(auto itr = prims.begin(); itr != prims.end(); itr++)
                                prim.push_back((*itr));
                            bbox = computeBounds(prim); 
                            return;
                        }
                    }
                };

                bool intersect(Ray& ray, Hit& res, const Vec3& invdir, const int dirIsNeg[3]) const {
                    bvh_intersection_count++;

                    //if this node is leaf
                    if(left == nullptr && right == nullptr) {
                        primitive_intersection_count++;
                        bool hit = prim_intersect(prim, ray, res);
                        if(hit) {
                            if(res.t < ray.tmax) {
                                ray.tmax = res.t;
                            }
                        }
                        return hit;
                    };

                    //ray hits node's bounding box?
                    if(!bbox.intersect2(ray, invdir, dirIsNeg))
                        return false;

                    ray.hit_count++;
                    ray.factor *= 0.99f;

                    Hit res_left;
                    Hit res_right;
                    bool hit_left, hit_right = false;
                    //if direction[axis] is positive, visit left child first
                    //else visit right child first
                    if(dirIsNeg[splitAxis] == 0) {
                        hit_left = left->intersect(ray, res_left, invdir, dirIsNeg);
                        hit_right = right->intersect(ray, res_right, invdir, dirIsNeg);
                    }
                    else {
                        hit_right = right->intersect(ray, res_right, invdir, dirIsNeg);
                        hit_left = left->intersect(ray, res_left, invdir, dirIsNeg);
                    }

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


    public:
        std::shared_ptr<BVH_node> bvh_root;

        BVH() {};
        BVH(std::vector<std::shared_ptr<Primitive>>& prims) {
            bvh_root = std::shared_ptr<BVH_node>(new BVH_node(prims, BVH_PARTITION_TYPE::SAH));
            std::cout << "BVH Construction Finished!" << std::endl;
            std::cout << "BVH nodes:" << node_count << std::endl;
            std::cout << "BVH leaf nodes:" << leaf_count << std::endl;
            std::cout << "BVH leaf/node:" << (float)leaf_count/node_count * 100 << "%" << std::endl;
            std::cout << "X Split Count:" << xsplit_count << std::endl;
            std::cout << "Y Split Count:" << ysplit_count << std::endl;
            std::cout << "Z Split Count:" << zsplit_count << std::endl;
        };

        bool intersect(Ray& ray, Hit& res) const {
            const Vec3 invdir = 1.0f/ray.direction;
            int dirIsNeg[3];
            dirIsNeg[0] = ray.direction.x < 0 ? 1 : 0;
            dirIsNeg[1] = ray.direction.y < 0 ? 1 : 0;
            dirIsNeg[2] = ray.direction.z < 0 ? 1 : 0;
            return bvh_root->intersect(ray, res, invdir, dirIsNeg);
        };
};


/*
class BVH_array {
    private:
        struct BVH_array_node {
            AABB aabb;
            std::shared_ptr<Primitive> prim;
            bool leaf;

            BVH_array_node(const AABB& aabb) : aabb(aabb), leaf(false) {};
            BVH_array_node(std::shared_ptr<Primitive> _prim) {
                prim = _prim;
                aabb = _prim->aabb();
                leaf = true;
            };

            bool intersect(const Ray& ray, Hit& res) const {
                if(leaf)
                    return prim->intersect(ray, res);
                else
                    return aabb.intersect(ray);
            }
        };

    public:
        std::shared_ptr<BVH_array_node>* nodes;
        int node_count;
        int leaf_count;

        BVH_array() {};
        BVH_array(std::vector<std::shared_ptr<Primitive>>& prims) {
            nodes = new std::shared_ptr<BVH_array_node>[4*prims.size()];
            node_count = 0;
            leaf_count = 0;
            makeBVHnode(prims, 0);
            std::cout << "BVH Construction Finished!" << std::endl;
            std::cout << "BVH nodes:" << node_count << std::endl;
            std::cout << "BVH leaf nodes:" << leaf_count << std::endl;
            std::cout << "BVH leaf/node:" << (float)leaf_count/node_count * 100 << "%" << std::endl;
        };

        void makeBVHnode(std::vector<std::shared_ptr<Primitive>>& prims, int node_index) {
            node_count++;

            //if primitives array is empty
            if(prims.size() == 0) {
                std::cerr << "prims is empty" << std::endl;
                std::exit(1);
            }
            //if primitives array has only one element
            //make current node as leaf
            if(prims.size() == 1) {
                leaf_count++;
                nodes[node_index] = std::shared_ptr<BVH_array_node>(new BVH_array_node(prims[0]));
                return;
            }

            //choose random splitting axis
            //and sort primitives by the chosen axis
            int axis = (int)(3*rnd());
            if(axis == 0) {
                std::sort(prims.begin(), prims.end(), [](std::shared_ptr<Primitive> x, std::shared_ptr<Primitive> y) {
                        return x->aabb().center.x < y->aabb().center.x;
                        });
            }
            else if(axis == 1) {
                std::sort(prims.begin(), prims.end(), [](std::shared_ptr<Primitive> x, std::shared_ptr<Primitive> y) {
                        return x->aabb().center.y < y->aabb().center.y;
                        });
            }
            else if(axis == 2) {
                std::sort(prims.begin(), prims.end(), [](std::shared_ptr<Primitive> x, std::shared_ptr<Primitive> y) {
                        return x->aabb().center.z < y->aabb().center.z;
                        });
            }

            //split primitives array in half
            std::size_t const half_size = prims.size()/2;
            std::vector<std::shared_ptr<Primitive>> left_prims(prims.begin(), prims.begin() + half_size);
            std::vector<std::shared_ptr<Primitive>> right_prims(prims.begin() + half_size, prims.end());

            //recursively call makeBVHnode for left and right children
            int left = getLeftIndex(node_index);
            int right = getRightIndex(node_index);
            makeBVHnode(left_prims, left); 
            makeBVHnode(right_prims, right); 

            //merge left and right children bounding box
            AABB mergedAABB = mergeAABB(nodes[left]->aabb, nodes[right]->aabb);
            
            //make current node
            nodes[node_index] = std::shared_ptr<BVH_array_node>(new BVH_array_node(mergedAABB));
        };
        //make nodes using Surface Area Heuristics(SAH)
        void makeBVHnode_SAH(std::vector<std::shared_ptr<Primitive>>& prims, int node_index) {
        };

        int getLeftIndex(int i) const {
            return 2*i + 1;
        };
        int getRightIndex(int i) const {
            return 2*i + 2;
        };

        bool intersect(const Ray& ray, Hit& res) const {
            return intersect_node(ray, res, 0);
        };
        bool intersect_node(const Ray& ray, Hit& res, int node_index) const {
            //if this node is leaf
            if(nodes[node_index]->leaf) {
                return nodes[node_index]->prim->intersect(ray, res);
            }

            //ray hits this node?
            if(!nodes[node_index]->aabb.intersect(ray))
                return false;

            //ray hits left or right chilren node?
            Hit res_left; 
            Hit res_right;
            bool hit_left = intersect_node(ray, res_left, getLeftIndex(node_index));
            bool hit_right = intersect_node(ray, res_right, getRightIndex(node_index));

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
            else {
                return false;
            }
        };
};
*/


class Primitives {
    public:
        std::vector<std::shared_ptr<Primitive>> prims;
        std::shared_ptr<BVH> bvh;

        Primitives() {};

        void add(Primitive* prim) {
            prims.push_back(std::shared_ptr<Primitive>(prim));
        };
        void loadObj(const Vec3& center, float scale, const std::string& filename) {
            tinyobj::attrib_t attrib;
            std::vector<tinyobj::shape_t> shapes;
            std::vector<tinyobj::material_t> materials;

            std::string err;
            bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str());
            if(!err.empty())
                std::cerr << err << std::endl;
            if(!ret)
                std::exit(1);

            int face_count = 0;
            int vertex_count = 0;
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
                        vertex.push_back(scale*Vec3((float)vx, (float)vy, (float)vz));
                        vertex_count++;
                    }
                    index_offset += fv;
                    prims.push_back(std::shared_ptr<Triangle>(new Triangle(center + vertex[0], center + vertex[1], center + vertex[2])));
                    face_count++;
                }
            }
            std::cout << "face:" << face_count << std::endl;
            std::cout << "vertex:" << vertex_count << std::endl;
        };

        void constructBVH() {
            bvh = std::shared_ptr<BVH>(new BVH(prims));
        };
        bool intersect(Ray& ray, Hit& res) const {
            return bvh->intersect(ray, res);
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
