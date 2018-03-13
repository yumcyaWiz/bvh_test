#ifndef IMAGE_H
#define IMAGE_H
#include <iostream>
#include <fstream>
#include <string>
#include "rgb.h"
#include "util.h"
class Image {
    public:
        int width;
        int height;
        RGB* data;

        Image(int width, int height) : width(width), height(height) {
            data = new RGB[width*height];
        };
        ~Image() {
            delete[] data;
        };

        RGB getPixel(int i, int j) const {
            return data[i + width*j];
        };
        void setPixel(int i, int j, const RGB& c) {
            data[i + width*j] = c;
        };

        void ppm_output(const std::string& filename) const {
            std::ofstream file(filename);
            
            file << "P3" << std::endl;
            file << width << " " << height << std::endl;
            file << 255 << std::endl;
            for(int j = 0; j < height; j++) {
                for(int i = 0; i < width; i++) {
                    RGB c = data[i + width*j];
                    file << clamp((int)(255*c.r), 0, 255) << " " << clamp((int)(255*c.g), 0, 255) << " " << clamp((int)(255*c.b), 0, 255) << std::endl;
                }
            }

            file.close();
        };
};
#endif
