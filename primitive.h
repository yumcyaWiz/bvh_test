#ifndef PRIMITIVE_H
#define PRIMITIVE_H
#include <vector>
#include <memory>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "vec3.h"
#include "ray.h"
#include "hit.h"
#include "aabb.h"
class Primitive {
    public:
        Vec3 center;

        Primitive() {};
        Primitive(const Vec3& center) : center(center) {};

        virtual bool intersect(const Ray& ray, Hit& hit) const = 0;
        virtual AABB aabb() const = 0;
};


class Sphere : public Primitive {
    public:
        float radius;

        Sphere(const Vec3& center, float radius) : Primitive(center), radius(radius) {};

        bool intersect(const Ray& ray, Hit& hit) const {
            float b = dot(ray.direction, ray.origin - center);
            float c = (ray.origin - center).length2() - radius*radius;
            float D = b*b - c;
            if(D < 0)
                return false;
            float t1 = -b - std::sqrt(D);
            float t2 = -b + std::sqrt(D);
            if(t1 > ray.tmax || t2 <= ray.tmin)
                return false;
            float tHit = t1;
            if(t1 < ray.tmin) {
                tHit = t2;
                if(tHit > ray.tmax)
                    return false;
            }

            hit.t = tHit;
            hit.hitPos = ray(tHit);
            hit.hitNormal = normalize(hit.hitPos - center);
            return true;
        };
        AABB aabb() const {
            return AABB(center - Vec3(radius), center + Vec3(radius));
        };
};


class Triangle : public Primitive {
    public:
        Vec3 p1;
        Vec3 p2;
        Vec3 p3;
        Vec3 normal;

        Triangle() {};
        Triangle(const Vec3& p1, const Vec3& p2, const Vec3& p3) : Primitive((p1 + p2 + p3)/3), p1(p1), p2(p2), p3(p3) {
            normal = normalize(cross(p2 - p1, p3 - p1));
        };

        //Moller-Trumbore Algorithm
        bool intersect(const Ray& ray, Hit& hit) const {
            const float eps = 0.000001;
            const Vec3 edge1 = p2 - p1;
            const Vec3 edge2 = p3 - p1;
            const Vec3 h = cross(ray.direction, edge2);
            const float a = dot(edge1, h);
            if(a >= -eps && a <= eps)
                return false;
            const float f = 1/a;
            const Vec3 s = ray.origin - p1;
            float u = f*dot(s, h);
            if(u < 0.0 || u > 1.0)
                return false;
            const Vec3 q = cross(s, edge1);
            const float v = f*dot(ray.direction, q);
            if(v < 0.0 || u + v > 1.0)
                return false;
            const float t = f*dot(edge2, q);
            if(t <= ray.tmin || t > ray.tmax)
                return false;
            
            hit.t = t;
            hit.hitPos = ray(t);
            hit.hitNormal = normal;
            return true;
        };
        AABB aabb() const {
            const float minX = std::min(std::min(p1.x, p2.x), p3.x);
            const float minY = std::min(std::min(p1.y, p2.y), p3.y);
            const float minZ = std::min(std::min(p1.z, p2.z), p3.z);
            const float maxX = std::max(std::max(p1.x, p2.x), p3.x);
            const float maxY = std::max(std::max(p1.y, p2.y), p3.y);
            const float maxZ = std::max(std::max(p1.z, p2.z), p3.z);
            return AABB(Vec3(minX, minY, minZ), Vec3(maxX, maxY, maxZ));
        };
};


class Polygon : public Primitive {
    public:
        Vec3 center;
        std::vector<std::shared_ptr<Triangle>> triangles;

        Polygon() {};
        //using tinyobj loader
        Polygon(const Vec3& center, const std::string& filename) : center(center) {
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
                    triangles.push_back(std::shared_ptr<Triangle>(new Triangle(center + vertex[0], center + vertex[1], center + vertex[2])));
                }
            }
        };

        bool intersect(const Ray& ray, Hit& res) const {
            bool hit = false;
            res.t = ray.tmax;
            for(auto itr = triangles.begin(); itr != triangles.end(); itr++) {
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
            Vec3 pMin(std::numeric_limits<float>::max());
            Vec3 pMax(std::numeric_limits<float>::lowest());
            for(auto itr = triangles.begin(); itr != triangles.end(); itr++) {
                Vec3 p1 = (*itr)->p1;
                Vec3 p2 = (*itr)->p2;
                Vec3 p3 = (*itr)->p3;
                
                pMin.x = std::min(pMin.x, std::min(std::min(p1.x, p2.x), p3.x));
                pMin.y = std::min(pMin.y, std::min(std::min(p1.y, p2.y), p3.y));
                pMin.z = std::min(pMin.z, std::min(std::min(p1.z, p2.z), p3.z));
                
                pMax.x = std::max(pMax.x, std::max(std::max(p1.x, p2.x), p3.x));
                pMax.y = std::max(pMax.y, std::max(std::max(p1.y, p2.y), p3.y));
                pMax.z = std::max(pMax.z, std::max(std::max(p1.z, p2.z), p3.z));
            }
            return AABB(pMin, pMax);
        };


        //test
        void test() const {
            for(auto itr = triangles.begin(); itr != triangles.end(); itr++) {
                std::cout << "p1:" << (*itr)->p1 << " p2:" << (*itr)->p2 << " p3:" << (*itr)->p3 << std::endl;
                std::cout << "normal:" << (*itr)->normal << std::endl;
            }
        };
};
#endif
