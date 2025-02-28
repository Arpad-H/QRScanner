//
// Created by quint on 2/11/2025.
//

#ifndef QRSCANNER_QRDECODER_H
#define QRSCANNER_QRDECODER_H

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <map>
#include "AviWrite.h"
#include "BmpRead.h"
#include "BmpWrite.h"
#include "ConvImg.h"
#include "FFT.h"
#include "Img.h"
#include "Vector.h"
#include "Entzerren.h"
#include "Matrix.h"


class QRdecoder {
public:
    QRdecoder() = default;

    void decodeQR(const string &filepath);

    Img<RGB_Pixel> readImage();

    Img<unsigned char> greyscale(Img<RGB_Pixel> img);

    Img<bool> binarization(const Img<unsigned char> &greyscale);

    unsigned int optimalThreshold(const Img<unsigned char> &greyscale);

    void writeImage(Img<bool> binary, string fileextension);

    vector<Vector> findFinderPatterns(const Img<bool> &binary_image);

    bool is_approx_ratio(const vector<int> &lengths, const vector<double> &target_ratio, double tolerance = 0.15);

    Img<unsigned char> medianBlur(const Img<unsigned char> &gray);

    void writeImage(Img<unsigned char> binary, string filename);

    void writeImage(Img<RGB_Pixel> binary, string filename);

    void visualizeVectors(vector<Vector> location, int thickness, bool border, int borderDistance);
    bool validate_vertical_ratio(const Img<bool> &binary_image, int x, int y);

    bool resize (Img<RGB_Pixel> &img);

private:
    int vizCounter = 0;
    int imageWidth;
    int imageHeight;
    string FILEPATH;
    string FILENAME;
    vector<Vector> locators = {Vector(0, 0, 0), Vector(0, 0, 0), Vector(0, 0, 0)}; // top left, top right, bottom left
//    bool detect_finder_pattern(const Img<bool> &binary_image, int y=0, int x=0);
};


#endif //QRSCANNER_QRDECODER_H
