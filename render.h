#ifndef RENDER_H
#define RENDER_H
#include "omp.h"
#include "vec3.h"
#include "rgb.h"
#include "image.h"
#include "camera.h"
#include "primitives.h"
class Render {
    public:
        Image* img;
        Camera* cam;
        Primitives* prims;
        int samples;

        Render(Image* _img, Camera* _cam, Primitives* _prims, int _samples) {
            img = _img;
            cam = _cam;
            prims = _prims;
            samples = _samples;
        };

        RGB Li(Ray& ray, int depth) const {
            if(depth > 10)
                return RGB(0);

            Hit res;
            if(prims->intersect(ray, res)) {
                Ray nextRay(res.hitPos, randomInUnitSphere());
                float k = std::max(dot(res.hitNormal, nextRay.direction), 0.0f);
                return k * Li(nextRay, depth + 1);
            }
            else {
                return RGB(1.0f);
            }
        };

        void render() {
            #pragma omp parallel for schedule(dynamic, 1)
            for(int k = 0; k < samples; k++) {
                for(int i = 0; i < img->width; i++) {
                    for(int j = 0; j < img->height; j++) {
                        float u = (2.0f*i - img->width + rnd())/img->width;
                        float v = (2.0f*j - img->height + rnd())/img->height;
                        Ray ray = cam->getRay(u, v);
                        img->setPixel(i, j, img->getPixel(i, j) + Li(ray, 0));
                    }
                }
                if(omp_get_thread_num() == 0)
                    std::cout << progressbar(k, samples) << " " << percentage(k, samples) << "\r" << std::flush;
            }
            img->divide(samples);
        };
        void render_normal() {
            #pragma omp parallel for schedule(dynamic, 1)
            for(int i = 0; i < img->width; i++) {
                for(int j = 0; j < img->height; j++) {
                    float u = (2.0f*i - img->width)/img->width;
                    float v = (2.0f*j - img->height)/img->height;
                    Ray ray = cam->getRay(u, v);
                    Hit res;
                    RGB col;
                    if(prims->intersect(ray, res)) {
                        col = RGB((res.hitNormal + 1.0f)/2.0f);
                    }
                    else {
                        col = RGB(0.0f);
                    }
                    img->setPixel(i, j, col);
                }
                if(omp_get_thread_num() == 0)
                    std::cout << progressbar(i, img->width) << " " << percentage(i, img->width) << "\r" << std::flush;
            }
        };

        void output() const {
            img->gamma_correction();
            img->ppm_output("output.ppm");
        };
};
#endif
