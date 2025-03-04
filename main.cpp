#include <omp.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <map>
#include <queue>
#include "AviWrite.h"
#include "BmpRead.h"
#include "BmpWrite.h"
#include "ConvImg.h"
#include "FFT.h"
#include "Img.h"
#include "Vector.h"
#include "Entzerren.h"
#include "Matrix.h"
#include "Eigen/Dense"



#define EPSILON  1e-9
#define PI 3.14159265358979323846


using namespace std;

int CurrentImageWidth = 0;
int CurrentImageHeight = 0;
Img<RGB_Pixel> vis; // Visualisierungsbild beim debuggen
string filepath;

struct ObjectProperties {
    Vector touch_point;
    unsigned int label;
    unsigned int x_min, x_max, y_min, y_max;
    unsigned int area;
    Vector center;
    RGB_Pixel color;
};

struct Line {
    double rho;
    double theta;
};


/*
 * Assigns the three objects to the three corners of a triangle
 * @param a first object
 * @param b second object
 * @param c third object
 * @out topLeft top left corner
 * @out top right corner
 * @out bottomLeft bottom left corner
 */
void AssignPropsToVectors(const ObjectProperties &a, const ObjectProperties &b, const ObjectProperties &c, Vector &topLeft,
                          Vector &topRight,
                          Vector &bottomLeft) {
    Vector av = a.center;
    Vector bv = b.center;
    Vector cv = c.center;

    Vector ab = bv - av;
    Vector ac = cv - av;
    Vector bc = cv - bv;


    float d1 = (bv - av).length(); // Distance AB
    float d2 = (cv - av).length(); // Distance AC
    float d3 = (cv - bv).length(); // Distance BC

    // Step 2: Identify the hypotenuse
    float max_d = max(d1, max(d2, d3));

    Vector midPoint;

    // with the hypotenuse we can determine the top left corner
    // A refrence point gets rotated around the top left corner
    // the point with the longer distance to the refrence point is the top right corner
    if (max_d == d1) { // AB is the hypotenuse
        midPoint = av + (ab * 0.5f);
        Vector cm = cv - midPoint;
        Matrix mrot;
        mrot.rotationAxis(Vector(0, 0, 1), PI / 2);
        cm = mrot * cm;
        cm = cm + cv; //refrence point
        vector<ObjectProperties> viz;

        if ((cm - av).length() > (cm - bv).length()) {
            topLeft = cv;
            topRight = av;
            bottomLeft = bv;
        } else {
            topLeft = cv;
            topRight = bv;
            bottomLeft = av;
        }

    } else if (max_d == d2) { // AC is the hypotenuse
        midPoint = av + (ac * 0.5f);
        Vector bm = bv - midPoint;
        Matrix mrot;
        mrot.rotationAxis(Vector(0, 0, 1), PI / 2);
        bm = mrot * bm;
        bm = bm + bv; //refrence point
        vector<ObjectProperties> viz;

        if ((bm - av).length() > (bm - cv).length()) {
            topLeft = bv;
            topRight = av;
            bottomLeft = cv;
        } else {
            topLeft = bv;
            topRight = cv;
            bottomLeft = av;
        }
    } else if (max_d == d3) { // BC is the hypotenuse
        midPoint = bv + (bc * 0.5f);
        Vector am = av - midPoint;
        Matrix mrot;
        mrot.rotationAxis(Vector(0, 0, 1), PI / 2);
        am = mrot * am;
        am = am + av; //refrence point
        if ((am - bv).length() > (am - cv).length()) {
            topLeft = av;
            topRight = bv;
            bottomLeft = cv;
        } else {
            topLeft = av;
            topRight = cv;
            bottomLeft = bv;
        }

    }
}


/*
 * Estimates the fourth corner of a rectangle
 * @param props vector of the three corners
 * @return ObjectProperties object with the estimated fourth corner
 */
ObjectProperties EstimtateFourthCorner(vector<ObjectProperties> props) {
    Vector topLeft;
    Vector topRight;
    Vector bottomLeft;
    Vector bottomRight;
    AssignPropsToVectors(props[0], props[1], props[2], topLeft, topRight, bottomLeft);

    int avgWidth =
            int(props[0].x_max - props[0].x_min + props[1].x_max - props[1].x_min + props[2].x_max - props[2].x_min) /
            3;
    avgWidth *= 1.5; //wiggle room
    bottomRight = bottomLeft + (topRight - topLeft);
    ObjectProperties obj;
    obj.center = bottomRight;
    obj.x_min = bottomRight.X - avgWidth / 2;
    obj.x_max = bottomRight.X + avgWidth / 2;
    obj.y_min = bottomRight.Y - avgWidth / 2;
    obj.y_max = bottomRight.Y + avgWidth / 2;
    return obj;
}

/*
 * Visualizes the points in the image
 * @param vector of points(Vektors) to visualize
 * @param desc description of the visualization
 * @param color color of the points
 */
void Visualize(const vector<Vector> &points, string desc, Vector color = Vector(255, 0, 255)) {

    string filename;
    for (const auto &point: points) {
        vis[point.Y][point.X] = RGB_Pixel(color.X, color.Y, color.Z);
        vis[point.Y - 1][point.X] = RGB_Pixel(color.X, color.Y, color.Z);
        vis[point.Y + 1][point.X] = RGB_Pixel(color.X, color.Y, color.Z);
        vis[point.Y][point.X - 1] = RGB_Pixel(color.X, color.Y, color.Z);
        vis[point.Y][point.X + 1] = RGB_Pixel(color.X, color.Y, color.Z);
    }

    try {
        filename = filepath + "_viz_" + desc + ".bmp";
        BmpWrite(filename.c_str(), vis);
        cout << "Schreibe viz Bild: " << filename << endl;
    }
    catch (const char *s) {
        cerr << "Fehler beim Schreiben von " << filename << ": " << s << endl;

    }

}

/*
 * Converts an RGB image to a greyscale image
 * @param img RGB image
 * @return greyscale image
 */
Img<unsigned char> greyscale(Img<RGB_Pixel> img) {
    Img<unsigned char> grey(img.Width(), img.Height());
    for (unsigned int y = 0; y < img.Height(); ++y) {
        for (unsigned int x = 0; x < img.Width(); ++x) {
            grey[y][x] = static_cast<unsigned char>(0.299 * img[y][x].Red() + 0.587 * img[y][x].Green() + 0.114 * img[y][x].Blue());
        }
    }
    return grey;
}

/*
 * Visualizes ObjectProperties in the image with their bounding box
 * @param vector of points(Vektors) to visualize
 * @param desc description of the visualization
 * @param color color of the points
 */
Img<RGB_Pixel> visualizeObjProps(const vector<ObjectProperties> &obj_props) {

    Img<RGB_Pixel> QRHighlited(CurrentImageWidth, CurrentImageHeight);

    for (const auto &obj: obj_props) {


        for (unsigned int y = obj.y_min; y <= obj.y_max; y++) {
            for (unsigned int x = obj.x_min; x <= obj.x_max; x++) {
                if (y == obj.y_min || y == obj.y_max || x == obj.x_min || x == obj.x_max) {
                    QRHighlited[y][x] = RGB_Pixel(255, 0, 0);
                    QRHighlited[obj.center.Y][obj.center.X] = RGB_Pixel(0, 255, 0);
                }
            }

        }

    }


    return QRHighlited;
}

/*
 * Checks if the object is a square based on the bounding box
 * @param obj ObjectProperties object
 * @param MIN_AREA minimum area of the object
 * @param MAX_AREA maximum area of the object
 * @return true if the object is a square
 */
bool isSquare(const ObjectProperties &obj, const unsigned int MIN_AREA, const unsigned int MAX_AREA) {
    unsigned int width = obj.x_max - obj.x_min + 1;
    unsigned int height = obj.y_max - obj.y_min + 1;

    if (abs((int) width - (int) height) > max(width, height) * 0.2) {
        return false;
    }

    if (obj.area < MIN_AREA || obj.area > MAX_AREA) {
        return false;
    }
    return true;
}

/*
 * Gets the properties of the objects in the labell image
 * @param label_image image with labeled objects
 * @return vector of ObjectProperties objects
 */
vector<ObjectProperties> getObjectProperties(const Img<unsigned int> &labelImage) {
    map<unsigned int, ObjectProperties> objects;

    for (unsigned int y = 0; y < labelImage.Height(); y++) {
        for (unsigned int x = 0; x < labelImage.Width(); x++) {
            unsigned int label = labelImage[y][x];
            if (label > 0) {
                auto &obj = objects[label];
                if (obj.area == 0) obj.touch_point = Vector(x, y, 0);
                obj.label = label;
                obj.x_min = (obj.x_min == 0) ? x : min(obj.x_min, x);
                obj.x_max = max(obj.x_max, x);
                obj.y_min = (obj.y_min == 0) ? y : min(obj.y_min, y);
                obj.y_max = max(obj.y_max, y);
                obj.area++;
            }
        }
    }

    // Calculate centers
    vector<ObjectProperties> results;
    for (auto &[label, obj]: objects) {
        float xc = (obj.x_min + obj.x_max) / 2;
        float yc = (obj.y_min + obj.y_max) / 2;
        obj.center = Vector(xc, yc, 0);
        results.push_back(obj);
    }

    return results;
}

/*
 * Function from Lecture
 * Labels the objects in the binary image
 * @param label_image image with labeled objects
 * @param touch_points vector of touch points
 * @param object_sizes vector of object sizes
 * @param connectivity connectivity of the objects
 * @param binary_image binary image
 * @return number of objects
 */
unsigned int Labelling(Img<unsigned int> &label_image, vector<pair<int, int> > &touch_points,
                       vector<unsigned int> &object_sizes, const unsigned int connectivity,
                       const Img<bool> &binary_image) {
    const Img<bool> &v16(binary_image);
    const unsigned int v0 = v16.Width();
    const unsigned int v1 = v16.Height();
    const unsigned int &v15(connectivity);
    if ((4 != v15) && (8 != v15)) {
        return -1;
    }
    unsigned int v2(0);
    vector<unsigned int> v3;
    Img<unsigned int> &v12(label_image);
    v12.Resize(v0, v1);
    v12.Margin_Constant(0);
    v3.push_back(0);
    for (unsigned int v4 = 0; v4 < v1; v4++) {
        for (unsigned int v5 = 0; v5 < v0; v5++) {
            v12[v4][v5] = 0;
        }
    }
    vector<pair<int, int> > &v13(touch_points);
    for (unsigned int v4 = 0; v4 < v1; v4++) {
        for (unsigned int v5 = 0; v5 < v0; v5++) {
            if (v16[v4][v5]) {
                vector<unsigned int> v6;
                if (unsigned int v11 = v12[v4 - 1][v5]) {
                    v6.push_back(v11);
                }
                if (unsigned int v11 = v12[v4][v5 - 1]) {
                    v6.push_back(v11);
                }
                if (8 == v15) {
                    if (unsigned int v11 = v12[v4 - 1][v5 - 1]) {
                        v6.push_back(v11);
                    }
                    if (unsigned int v11 = v12[v4 - 1][v5 + 1]) {
                        v6.push_back(v11);
                    }
                }
                if (0 == v6.size()) {
                    v2++;
                    v12[v4][v5] = v2;
                    v3.push_back(v2);
                    v13.push_back(pair<int, int>(v5, v4));
                } else {
                    unsigned int v7 = v6.at(0);
                    for (unsigned int v10 = 0; v10 < v6.size(); v10++) {
                        unsigned int v8 = v6.at(v10);
                        if (v8 < v7) {
                            v7 = v8;
                        }
                    }
                    if (v3.at(v7) < v7) {
                        v7 = v3.at(v7);
                    }
                    v12[v4][v5] = v7;
                    for (unsigned int v18 = 0; v18 < v6.size(); v18++) {
                        unsigned int v8 = v6.at(v18);
                        v3.at(v8) = v7;
                    }
                }
            }
        }
    }
    for (unsigned int v17 = 0; v17 < v3.size(); v17++) {
        unsigned int v18 = v17;
        while (v18 != v3[v18]) {
            v18 = v3[v18];
        }
        v3[v17] = v18;
    }
    v2 = 0;
    for (unsigned int i = 0; i < v3.size(); i++) {
        if (v3[i] > v2) {
            v2++;
            unsigned int v9 = v3[i];
            for (unsigned int j = i; j < v3.size(); j++) {
                if (v3[j] == v9) {
                    v3[j] = v2;
                }
            }
            v13[v2 - 1] = v13[v9 - 1];
        }
    }
    vector<unsigned int> &v14(object_sizes);
    v14.resize(v2, 0);
    for (unsigned int v4 = 0; v4 < v1; v4++) {
        for (unsigned int v5 = 0; v5 < v0; v5++) {
            v12[v4][v5] = v3[v12[v4][v5]];
            if (v12[v4][v5] > 0) {
                v14[v12[v4][v5] - 1] += 1;
            }
        }
    }
    return v2;
}


/*
 * colors the label image to an RGB image
 * @param label_image image with labeled objects
 * @param colors vector of colors
 * @return RGB image
 */
Img<RGB_Pixel> Labelimage_to_RGB(const Img<unsigned int> &label_image, vector<RGB_Pixel> colors) {
    const unsigned int width = label_image.Width();
    const unsigned int height = label_image.Height();
    Img<RGB_Pixel> Label_RGB(width, height);
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            const unsigned int &label = label_image[y][x];
            if (label == 0) {
                Label_RGB[y][x] = RGB_Pixel(255, 255, 255);
            } else {
                RGB_Pixel &color = colors[label - 1];
                Label_RGB[y][x] = color;
            }
        }
    }
    return Label_RGB;
}

/*
 * Finds the potential finder patterns in the image
 * @param obj_props vector of ObjectProperties objects
 * @return vector of potential finder patterns
 */
vector<ObjectProperties> getPotentialFinderPatterns(const vector<ObjectProperties> &obj_props) {
    vector<ObjectProperties> potentialFinderPatterns;
    for (const auto &obj: obj_props) {
        if (isSquare(obj, 50, 10000)) { //TODO: adjust the area in a automatic way
            potentialFinderPatterns.push_back(obj);
        }
    }
    return potentialFinderPatterns;
}

/*
 * Checks if the 3 objects form a L shape
 * @param a first object
 * @param b second object
 * @param c third object
 * @return true if the objects form a L shape
 */
bool IsLShape(const ObjectProperties &a, const ObjectProperties &b, const ObjectProperties &c) {

    Vector topLeft;
    Vector topRight;
    Vector bottomLeft;
    AssignPropsToVectors(a, b, c, topLeft, topRight, bottomLeft);


    float dot = ((topLeft - topRight).normalize()).dot((topLeft - bottomLeft).normalize());

    if (dot - 0.1 < 0 && dot + 0.1 > 0) {
        if ((topLeft - topRight).length() / (topLeft - bottomLeft).length() > 0.9 &&
            (topLeft - topRight).length() / (topLeft - bottomLeft).length() < 1.1) {
            return true;
        }

    }
    return false;

}

/*
 * Assigns the three objects to the three corners of a triangle. similar to AssignPropstoVectors but with ObjectProperties
 * @param a first object
 * @param b second object
 * @param c third object
 * @out topLeft top left corner
 * @out top right corner
 * @out bottomLeft bottom left corner
 */
void AssignPropstoProps(const ObjectProperties &a, const ObjectProperties &b, const ObjectProperties &c,
                        ObjectProperties &topLeft,
                        ObjectProperties &topRight,
                        ObjectProperties &bottomLeft) {
    Vector av = a.center;
    Vector bv = b.center;
    Vector cv = c.center;


    float d1 = (bv - av).length(); // Distance AB
    float d2 = (cv - av).length(); // Distance AC
    float d3 = (cv - bv).length(); // Distance BC



    float max_d = max(d1, max(d2, d3));
    Vector midPoint;

    if (max_d == d1) { // AB is the hypotenuse
        midPoint = (av + bv) * 0.5f;
        if ((cv - midPoint).length() > (av - midPoint).length()) {
            topLeft = c;
            topRight = b;
            bottomLeft = a;
        } else {
            topLeft = c;
            topRight = a;
            bottomLeft = b;
        }

    } else if (max_d == d2) { // AC is the hypotenuse
        midPoint = (av + cv) * 0.5f;
        if ((bv - midPoint).length() > (av - midPoint).length()) {
            topLeft = b;
            topRight = a;
            bottomLeft = c;
        } else {
            topLeft = b;
            topRight = c;
            bottomLeft = a;
        }
    } else if (max_d == d3) { // BC is the hypotenuse
        midPoint = (bv + cv) * 0.5f;
        if ((av - midPoint).length() > (bv - midPoint).length()) {
            topLeft = a;
            topRight = b;
            bottomLeft = c;
        } else {
            topLeft = a;
            topRight = c;
            bottomLeft = b;
        }
    }
}

/*
 * Finds the L shaped patterns in the image
 * @param obj_props vector of ObjectProperties objects
 * @return vector of L shaped patterns
 */
vector<ObjectProperties> FindLShapedPatterns(const vector<ObjectProperties> &obj_props) {

    vector<ObjectProperties> bestPatterns;
    float bestScore = 0;
    for (int i = 0; i < obj_props.size(); i++) {
        for (int j = i + 1; j < obj_props.size(); j++) {
            for (int k = j + 1; k < obj_props.size(); k++) {

                if (IsLShape(obj_props[i], obj_props[j], obj_props[k])) {
                    float smallest = min(obj_props[i].area, min(obj_props[j].area, obj_props[k].area));

                    float avgArea = (obj_props[i].area + obj_props[j].area + obj_props[k].area) / 3;

                    if (smallest < avgArea * 0.8) {
                        continue;
                    }

                    float score = smallest * 3;
                    if (score > bestScore) {
                        bestScore = score;
                        bestPatterns = {obj_props[i], obj_props[j], obj_props[k]};
                    }

                }
            }
        }
    }
    return bestPatterns;
}

/*
 * Returns the angle of the L shaped pattern so that top left and top right are aligned with the x-axis
 * @param a first object
 * @param b second object
 * @param c third object
 * @return angle of the L shaped pattern
 */
float IsRotated(const ObjectProperties &a, const ObjectProperties &b, const ObjectProperties &c) {

    Vector topLeft, topRight, bottomLeft;
    AssignPropsToVectors(a, b, c, topLeft, topRight, bottomLeft);

    Vector horizontal = Vector(1, 0, 0);
    Vector topEdge = topRight - topLeft;
    topEdge.normalize();

    float angle = acos(horizontal.dot(topEdge));
    if (topEdge.Y < 0) {
        angle = -angle;
    }
    return angle;
}

/*
 * Creates the RGB colors for the labels
 * @param num_objects number of objects
 * @return vector of RGB colors
 */
vector<RGB_Pixel> create_LabelColors(unsigned int num_objects) {
    vector<RGB_Pixel> colors; // enthaelt Farben in Reihenfolge des Farbwinkels

    for (unsigned int i = 0; i < num_objects; i++) {
        double phase((2.0 / 3.0) * 2.0 * M_PI * double(i) / double(num_objects - 1));
        unsigned int blue = static_cast<unsigned int>(
                255 * (sin(phase + M_PI / 2.0 + 0.0 * 2 * M_PI / 3.0) + 1.0) / 2.0 + 0.5);
        unsigned int green = static_cast<unsigned int>(
                255 * (sin(phase + M_PI / 2.0 + 2.0 * 2 * M_PI / 3.0) + 1.0) / 2.0 + 0.5);
        unsigned int red = static_cast<unsigned int>(
                255 * (sin(phase + M_PI / 2.0 + 1.0 * 2 * M_PI / 3.0) + 1.0) / 2.0 + 0.5);
        colors.push_back(RGB_Pixel(red, green, blue));
    }

    random_shuffle(colors.begin(), colors.end());
    return colors;
}


template<typename VT>
vector<RGB_Pixel> Values2ColorVector(const vector<VT> values) {
    // Farbe wird abhaengig von den Merkmalswerten vergeben
    vector<VT> sort_values(values);
    sort(sort_values.begin(), sort_values.end());
    VT Max(values[0]);
    for (unsigned int i = 1; i < values.size(); i++) {
        if (values[i] > Max) {
            Max = values[i];
        }
    }
    map<VT, unsigned int> size_to_int;
    vector<RGB_Pixel> sort_colors;
    for (unsigned int i = 0; i < sort_values.size(); i++) {
        double phase((2.0 / 3.0) * 2.0 * M_PI * double(sort_values[i]) / double(Max));
        unsigned int blue = static_cast<unsigned int>(
                255 * (sin(phase + M_PI / 2.0 + 0.0 * 2 * M_PI / 3.0) + 1.0) / 2.0 + 0.5);
        unsigned int green = static_cast<unsigned int>(
                255 * (sin(phase + M_PI / 2.0 + 2.0 * 2 * M_PI / 3.0) + 1.0) / 2.0 + 0.5);
        unsigned int red = static_cast<unsigned int>(
                255 * (sin(phase + M_PI / 2.0 + 1.0 * 2 * M_PI / 3.0) + 1.0) / 2.0 + 0.5);
        sort_colors.push_back(RGB_Pixel(red, green, blue));
        size_to_int[sort_values[i]] = i;
    }
    // Farben werden nun nach Labelnummern sortiert
    vector<RGB_Pixel> colors;
    for (unsigned int i = 0; i < values.size(); i++) {
        colors.push_back(sort_colors[size_to_int[values[i]]]);
    }
    return colors;
}

/*
 * Creates a mirrored Structuring Element
 * @param ImageWindow original Structuring Element
 * @return mirrored Structuring Element
 */
vector<Position> mirror_SE(const vector<Position> &ImageWindow) {
    vector<Position> MirroredImageWindow(ImageWindow.size());

    for (const auto &pos: ImageWindow) {
        MirroredImageWindow.push_back(Position{-pos.get_x(), -pos.get_y()});
    }

    return MirroredImageWindow;
}

/*
 * closes the image
 * @param src source image
 * @param ImageWindow Structuring Element
 * @return closed image
 */
template<typename Pixel>
Img<Pixel> closing(const Img<Pixel> &src, const vector<Position> &ImageWindow) {
    Img<Pixel> closed;

    Img<Pixel> dilated = dilate(src, ImageWindow);


    std::vector<Position> mirrored_SE = mirror_SE(ImageWindow);
    closed = erode(dilated, mirrored_SE);

    return closed;
}

/*
 * Subtracts two images
 * @param l left image
 * @param r right image
 * @return difference image
 */
template<typename Pixel>
Img<Pixel> operator-(const Img<Pixel> &l, const Img<Pixel> &r) {
    Img<Pixel> d(l.Width(), l.Height());


    for (unsigned int y = 0; y < l.Height(); ++y) {
        for (unsigned int x = 0; x < l.Width(); ++x) {
            // Berechnung der Differenz der Pixelwerte (Clamping bei Bedarf)
            d[y][x] = std::max(0, l[y][x] - r[y][x]);
        }
    }

    return d;
}

/*
 * Opens the image
 * @param src source image
 * @param ImageWindow Structuring Element
 * @return opened image
 */
template<typename Pixel>
Img<Pixel> opening(const Img<Pixel> &src, const vector<Position> &ImageWindow) {
    Img<Pixel> opened;

    Img<Pixel> eroded = erode(src, ImageWindow);


    std::vector<Position> mirrored_SE = mirror_SE(ImageWindow);
    opened = dilate(eroded, mirrored_SE);

    return opened;
}

/*
 * Erodes the objects in the image
 * @param src source image
 * @param ImageWindow Structuring Element
 * @return eroded image
 */
template<typename Pixel>
Img<Pixel> erode(const Img<Pixel> &src, const vector<Position> &ImageWindow) {
    Img<Pixel> eroded(src.Width(), src.Height());
    const unsigned int Width = src.Width();
    const unsigned int Height = src.Height();


    for (unsigned int y = 0; y < Height; ++y) {
        for (unsigned int x = 0; x < Width; ++x) {
            Pixel min_value = std::numeric_limits<Pixel>::max();
            for (const auto &pos: ImageWindow) {
                int nx = x + pos.get_x();
                int ny = y + pos.get_y();

                if (nx >= 0 && nx < Width && ny >= 0 && ny < Height) {
                    min_value = std::min(min_value, src[ny][nx]);
                }
            }
            eroded[y][x] = min_value;
        }
    }

    return eroded;
}

/*
 * Dilates the objects in the image
 * @param src source image
 * @param ImageWindow Structuring Element
 * @return dilated image
 */
template<typename Pixel>
Img<Pixel> dilate(const Img<Pixel> &src, const vector<Position> &ImageWindow) {
    const unsigned int Width = src.Width();
    const unsigned int Height = src.Height();
    Img<Pixel> dilated(src.Width(), src.Height());


    for (unsigned int y = 0; y < Height; ++y) {
        for (unsigned int x = 0; x < Width; ++x) {
            Pixel max_value = std::numeric_limits<Pixel>::min();
            for (const auto &pos: ImageWindow) {
                int nx = x + pos.get_x();
                int ny = y + pos.get_y();

                if (nx >= 0 && nx < Width && ny >= 0 && ny < Height) {
                    max_value = std::max(max_value, src[ny][nx]);
                }
            }
            dilated[y][x] = max_value;
        }
    }

    return dilated;
}

/*
 * Creates a round Structuring Element
 * @param Diameter Diameter of the Structuring Element
 * @return round Structuring Element
 */
vector<Position> create_round_SE(int Diameter) {
    vector<Position> ImageWindow;


    int Radius = Diameter / 2;
    int RadiusSquared = Radius * Radius;


    for (int y = -Radius; y <= Radius; ++y) {
        for (int x = -Radius; x <= Radius; ++x) {
            if (x * x + y * y <= RadiusSquared) {
                ImageWindow.push_back(Position{x, y});
            }
        }
    }


    return ImageWindow;
}

/*
 * Performs a median blur on the image
 * @param gray greyscale image
 * @return blurred image
 */
Img<unsigned char> medianBlur(const Img<unsigned char> &gray) {
    const unsigned int width = gray.Width();
    const unsigned int height = gray.Height();
    Img<unsigned char> blur(width, height);

    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            unsigned int sum = 0;
            unsigned int count = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (x + dx >= 0 && x + dx < width && y + dy >= 0 && y + dy < height) {
                        sum += gray[y + dy][x + dx];
                        ++count;
                    }
                }
            }
            blur[y][x] = sum / count;
        }
    }

    return blur;
}

/*
 *
 * Draws the lines in the image
 * @param image image to draw the lines on
 * @param lines vector of lines
 */
void DrawLines(Img<bool> &image, const vector<Line> &lines) {

    //ChatGPT prompt: How do you draw lines from hough space in a binary image
    for (const auto &line: lines) {
        double radian = line.theta;
        double cos_theta = cos(radian);
        double sin_theta = sin(radian);

        for (unsigned int y = 0; y < CurrentImageHeight; ++y) {
            for (unsigned int x = 0; x < CurrentImageWidth; ++x) {
                double rho = x * cos_theta + y * sin_theta;
                if (abs(rho - line.rho) < 1) {
                    image[y][x] = true;
                }
            }
        }
    }
}

/*
 * Iterative Thresholding idea from ChatGPT prompt: whats a good way to threshold a qr code image
 * calculates the optimal threshold for binarization
 * @param gray_image greyscale image
 * @return optimal threshold
 */
unsigned int optimal_threshold(const Img<unsigned char> &gray_image) {


    unsigned int T = 128;
    unsigned int prev_T;

    do {
        prev_T = T;
        unsigned int sum1 = 0, sum2 = 0;
        unsigned int count1 = 0, count2 = 0;

        for (unsigned int y = 0; y < CurrentImageHeight; ++y) {
            for (unsigned int x = 0; x < CurrentImageWidth; ++x) {
                unsigned char pixel = gray_image[y][x];
                if (pixel < T) {
                    sum1 += pixel;
                    count1++;
                } else {
                    sum2 += pixel;
                    count2++;
                }
            }
        }

        unsigned int V1 = (count1 == 0) ? 0 : (sum1 / count1);
        unsigned int V2 = (count2 == 0) ? 0 : (sum2 / count2);

        T = (V1 + V2) / 2;
    } while (T != prev_T);

    return T;

}

/*
 * Binarizes the greyscale image
 * @param gray_image greyscale image
 * @return binary image
 */
Img<bool> binarize(const Img<unsigned char> &gray_image) {

    Img<bool> binary_image(CurrentImageWidth,CurrentImageHeight);

    unsigned int T = optimal_threshold(gray_image);

    for (unsigned int y = 0; y < CurrentImageHeight; ++y) {
        for (unsigned int x = 0; x < CurrentImageWidth; ++x) {
            binary_image[y][x] = gray_image[y][x] > T;
        }
    }
    return binary_image;
}

/*
 * Performs Edge Detection on the binary image
 * @param binary binary image
 * @return edge detected image
 */
Img<bool> edgeDetection(const Img<bool> &binary) {
    const unsigned int width = binary.Width();
    const unsigned int height = binary.Height();
    Img<bool> edges(width, height);

    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            if (binary[y][x]) {
                edges[y][x] = false;
                continue;
            }
            bool edge = false;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) continue;
                    if (binary[y + dy][x + dx]) {
                        edge = true;
                        break;
                    }
                }
                if (edge) break;
            }
            edges[y][x] = edge;
        }
    }

    return edges;
}

/*
 * Extracts the strongest lines from the Hough space
 * @param hough Hough space
 * @param rho_max maximum rho value
 * @param theta_dim theta dimension
 * @return vector of lines
 */
vector<Line>
ExtractStrongestLines(const vector<vector<unsigned int>> &hough, unsigned int rho_max, unsigned int theta_dim) {


    int sum = 0;
    int count = 0;
    for (unsigned int rho = 0; rho < hough.size(); ++rho) {
        for (unsigned int theta = 0; theta < theta_dim; ++theta) {
            sum += hough[rho][theta];
            if (hough[rho][theta] > 0) count++;
        }
    }
    unsigned int threshold = (sum / count) * 2.6; //TODO Find a more sophisticated way to determine the threshold
    cout << "threshold: " << threshold << endl;

    vector<Line> detected_lines;

    // Parallelization done by ChatGPT. Prompt:  for (unsigned int rho = 0; rho < hough.size(); ++rho) {
    //            for (unsigned int theta = 0; theta < theta_dim; ++theta) {
    //                if (hough[rho][theta] > threshold &&
    //                    ((theta < 10 || (theta > 80 && theta < 100) || (theta > 170 && theta < 190) ||
    //                      (theta > 260 && theta < 280) || theta > 350))) {
    //                    detected_lines.push_back({(double) (rho - rho_max), theta * M_PI / 180.0});
    //                }
    //            }
    //        } parallelise this in c++
#pragma omp parallel
    {
        vector<Line> local_lines; // Thread-local storage to reduce contention

#pragma omp for
        for (unsigned int rho = 0; rho < hough.size(); ++rho) {
            for (unsigned int theta = 0; theta < theta_dim; ++theta) {
                if (hough[rho][theta] > threshold &&
                    ((theta < 10 || (theta > 80 && theta < 100) || (theta > 170 && theta < 190) ||
                      (theta > 260 && theta < 280) || theta > 350))) {
                    local_lines.push_back({(double) (rho - rho_max), theta * M_PI / 180.0});
                }
            }
        }

        // Merge local_lines into the main detected_lines vector in a thread-safe way
#pragma omp critical
        detected_lines.insert(detected_lines.end(), local_lines.begin(), local_lines.end());
    }
    cout << "Detected lines: " << detected_lines.size() << endl;
    return detected_lines;
}

/*
 * Determines if a line intersects a box
 * @param line line
 * @param box box
 * @param error_margin error margin
 * @return true if the line intersects the box
 */
bool doesLineIntersectBox(const Line &line, const ObjectProperties &box, float error_margin = 10.0) {

    double rho = line.rho;
    double theta = line.theta;
    double cosTheta = cos(theta), sinTheta = sin(theta);


    double x_min = box.x_min - error_margin;
    double x_max = box.x_max + error_margin;
    double y_min = box.y_min - error_margin;
    double y_max = box.y_max + error_margin;

    // vertical lines
    if (abs(sinTheta) < 1e-6) {
        double x = rho / cosTheta;
        return (x >= x_min && x <= x_max);
    }

    // horizontal lines
    if (abs(cosTheta) < 1e-6) {
        double y = rho / sinTheta;
        return (y >= y_min && y <= y_max);
    }

    //CahtGPT prompt: How do you check if a line in hough space intersects a box with its bounding box
    // Compute intersections
    double y_left = (-cosTheta / sinTheta) * x_min + (rho / sinTheta);
    double y_right = (-cosTheta / sinTheta) * x_max + (rho / sinTheta);
    double x_top = (-sinTheta / cosTheta) * y_min + (rho / cosTheta);
    double x_bottom = (-sinTheta / cosTheta) * y_max + (rho / cosTheta);

    // Check if any intersection is inside the expanded box
    bool intersectsVertically = (y_left >= y_min && y_left <= y_max) ||
                                (y_right >= y_min && y_right <= y_max);
    bool intersectsHorizontally = (x_top >= x_min && x_top <= x_max) ||
                                  (x_bottom >= x_min && x_bottom <= x_max);

    return intersectsVertically || intersectsHorizontally;
}

/*
 * Refines the QR code boundaries
 * @param lines vector of lines
 * @param qrCodeL vector of Object properties representing the bounding boxes of the finder patterns
 * @return vector of corners of the QR code
 */
vector<Vector> refineQRBounds(const vector<Line> &lines, const vector<ObjectProperties> &qrCodeL) {
    ObjectProperties topLeft, topRight, bottomLeft;
    AssignPropstoProps(qrCodeL[0], qrCodeL[1], qrCodeL[2], topLeft, topRight, bottomLeft);
    ObjectProperties bottomRight = qrCodeL[3];
    vector<Line> filteredLines;
    vector<Vector> intersectionPointsTopLeft;
    vector<Vector> intersectionPointsTopRight;
    vector<Vector> intersectionPointsBottomLeft;
    vector<Vector> intersectionPointsBottomRight;

    //parallelization done by ChatGPT. Prompt: for (auto &line: lines) {
    //        if (doesLineIntersectBox(line, qrCodeL[0], 50) || doesLineIntersectBox(line, qrCodeL[1], 50) ||
    //            doesLineIntersectBox(line, qrCodeL[2], 50)) {
    //            filteredLines.push_back(line);
    //        }
    //
    //    } parallelise this in c++
#pragma omp parallel
    {
        std::vector<Line> localFilteredLines; // Thread-local storage to avoid race conditions
#pragma omp for nowait
        for (size_t i = 0; i < lines.size(); ++i) {
            if (doesLineIntersectBox(lines[i], qrCodeL[0], 50) ||
                doesLineIntersectBox(lines[i], qrCodeL[1], 50) ||
                doesLineIntersectBox(lines[i], qrCodeL[2], 50)) {
                localFilteredLines.push_back(lines[i]);
            }
        }
        // Merge local results into the global vector safely
#pragma omp critical
        filteredLines.insert(filteredLines.end(), localFilteredLines.begin(), localFilteredLines.end());
    }

    for (auto &line: filteredLines) {
        if (doesLineIntersectBox(line, topRight) && doesLineIntersectBox(line, bottomRight)) {
            filteredLines.push_back(line);
        }
    }

    //Determine the most extreme lines
    Line leftMostLine, rightMostLine, topMostLine, bottomMostLine;
    bool leftFound = false, rightFound = false, topFound = false, bottomFound = false;

    const double verticalTolerance = 10 * (M_PI / 180); // 10-degree tolerance
    const double horizontalTolerance = 10 * (M_PI / 180); // 10-degree tolerance

    for (auto &line: filteredLines) {
        double rho = line.rho;
        double theta = line.theta;

        if (theta < verticalTolerance || theta > M_PI - verticalTolerance) {
            if (!leftFound || rho < leftMostLine.rho) {
                leftMostLine = line;
                leftFound = true;
            }
            if (!rightFound || rho > rightMostLine.rho) {
                rightMostLine = line;
                rightFound = true;
            }
        } else if (theta > M_PI / 2 - horizontalTolerance && theta < M_PI / 2 + horizontalTolerance) {
            if (!topFound || rho < topMostLine.rho) {
                topMostLine = line;
                topFound = true;
            }
            if (!bottomFound || rho > bottomMostLine.rho) {
                bottomMostLine = line;
                bottomFound = true;
            }
        }
    }

    vector<Line> edges = {topMostLine, bottomMostLine, leftMostLine, rightMostLine};

    //Determining the corners of the QR code
    vector<Vector> corners;
    for (int i = 0; i < edges.size(); i++) {
        for (int j = i + 1; j < edges.size(); j++) {
            Line L1 = edges[i];
            Line L2 = edges[j];
            double A1 = cos(L1.theta), B1 = sin(L1.theta), C1 = L1.rho;
            double A2 = cos(L2.theta), B2 = sin(L2.theta), C2 = L2.rho;


            // Compute determinant
            double det = A1 * B2 - A2 * B1;
            if (fabs(det) < 1e-6) continue; // Parallel lines

            // Solve for x, y
            double x = (C1 * B2 - C2 * B1) / det;
            double y = (A1 * C2 - A2 * C1) / det;

            if (x >= 0 && x <= CurrentImageWidth && y >= 0 && y <= CurrentImageHeight) {
                Vector intersectionPoint = Vector(x, y, 0);
                corners.push_back(intersectionPoint);

            }

        }
    }


    {
        Img<bool> hough_vis(CurrentImageWidth, CurrentImageHeight);
        for (unsigned int y = 0; y < CurrentImageHeight; ++y) {
            for (unsigned int x = 0; x < CurrentImageWidth; ++x) {
                hough_vis[y][x] = false;
            }
        }
        DrawLines(hough_vis, edges);
        try {
            string t = filepath + "_houghtest.bmp";
            BmpWrite(t.c_str(), hough_vis);
            cout << "Schreibe" << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von hough.bmp: " << strerror(errno) << endl;
        }
    }


    //Visualize(corners);
    return corners;

}

/*
 * Performs the Hough Transform on the binary image
 * @param input binary image
 * @param qrCodeL vector of Object properties representing the bounding boxes of the finder patterns
 * @return vector of corners of the QR code
 */
vector<Vector> HoughTransform(const Img<bool> &input, const vector<ObjectProperties> &qrCodeL) {


    const unsigned int rho_max = static_cast<unsigned int>(sqrt(
            CurrentImageWidth * CurrentImageWidth + CurrentImageHeight * CurrentImageHeight));
    const unsigned int rho_dim = 2 * rho_max + 1;
    const unsigned int theta_dim = 360;

    vector<vector<unsigned int>> hough(rho_dim, vector<unsigned int>(theta_dim, 0));


    unsigned int minY = 100000;
    unsigned int maxY = 0;
    unsigned int minX = 100000;
    unsigned int maxX = 0;

    for (ObjectProperties qrCode: qrCodeL) {
        minX = min(minX, qrCode.x_min);
        minY = min(minY, qrCode.y_min);
        maxX = max(maxX, qrCode.x_max);
        maxY = max(maxY, qrCode.y_max);
    }
    double expansionX = (qrCodeL[0].x_max - qrCodeL[0].x_min) * 1.5;
    double expansionY = (qrCodeL[0].y_max - qrCodeL[0].y_min) * 1.5;


    minX = max(minX - expansionX, 0.0);
    minY = max(minY - expansionY, 0.0);

    maxX = min(maxX + expansionX, (double) CurrentImageWidth);
    maxY = min(maxY + expansionY, (double) CurrentImageHeight);

    // Hough Transform from ChatGPT prompt: How do you perform hough transform on a binary image in c++
    for (unsigned int y = minY; y < maxY; ++y) {
        for (unsigned int x = minX; x < maxX; ++x) {
            if (input[y][x]) {
                for (int theta = 0; theta < theta_dim; ++theta) {
                    double radian = theta * M_PI / 180.0;
                    double rho = x * cos(radian) + y * sin(radian);
                    int rho_index = static_cast<int>(rho + rho_max);
                    if (rho_index >= 0 && rho_index < static_cast<int>(rho_dim)) {
                        hough[rho_index][theta]++;
                    }
                }
            }
        }
    }

    // Find strongest lines
    vector<Line> lines = ExtractStrongestLines(hough, rho_max, theta_dim);
    vector<Vector> corners = refineQRBounds(lines, qrCodeL);


    return corners;
}

/*
 * ChatGPT prompt: Matrix computeHomography(const vector<Vector> &corners, const vector<Vector> &correctedCorners) {
 * } how do i compute homography given 4 points with xy coordinates. i dont want to use a library like opencv
 *
 * Calculates the Homography matrix
 * @param corners vector of corners
 * @param correctedCorners vector of corrected corners
 * @return Homography matrix
 */
Eigen::Matrix3d computeHomography(const vector<Vector> &corners, const vector<Vector> &correctedCorners) {
    Eigen::MatrixXd A(8, 8);
    Eigen::VectorXd b(8);

    for (int i = 0; i < 4; i++) {
        double x = corners[i].X;
        double y = corners[i].Y;
        double x_p = correctedCorners[i].X;
        double y_p = correctedCorners[i].Y;

        A(2 * i, 0) = x;
        A(2 * i, 1) = y;
        A(2 * i, 2) = 1;
        A(2 * i, 3) = 0;
        A(2 * i, 4) = 0;
        A(2 * i, 5) = 0;
        A(2 * i, 6) = -x * x_p;
        A(2 * i, 7) = -y * x_p;
        b(2 * i) = x_p;

        A(2 * i + 1, 0) = 0;
        A(2 * i + 1, 1) = 0;
        A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = x;
        A(2 * i + 1, 4) = y;
        A(2 * i + 1, 5) = 1;
        A(2 * i + 1, 6) = -x * y_p;
        A(2 * i + 1, 7) = -y * y_p;
        b(2 * i + 1) = y_p;
    }

    Eigen::VectorXd h = A.colPivHouseholderQr().solve(b);

    Eigen::Matrix3d H;
    H << h(0), h(1), h(2),
            h(3), h(4), h(5),
            h(6), h(7), 1.0;

    return H;
}
/*
 * Converts Eigen Matrix to Matrix
 * @param matrix Eigen Matrix
 * @return Matrix
 */
Matrix EigneMatrix3dToMatrix(Eigen::Matrix3d matrix) {

    Matrix m;
    m.identity();
    m.m00 = matrix(0, 0);
    m.m01 = matrix(0, 1);
    m.m02 = matrix(0, 2);
    m.m10 = matrix(1, 0);
    m.m11 = matrix(1, 1);
    m.m12 = matrix(1, 2);
    m.m20 = matrix(2, 0);
    m.m21 = matrix(2, 1);
    m.m22 = matrix(2, 2);

    // m.print();
    return m;

}

/*
 * Determines the corners of the QR code
 * @param p1 first corner
 * @param p2 second corner
 * @param p3 third corner
 * @param p4 fourth corner
 * @param topLeft top left corner
 * @param topRight top right corner
 * @param bottomLeft bottom left corner
 * @param bottomRight bottom right corner
 */
void DetermineCorners(Vector &p1, Vector &p2, Vector &p3, Vector &p4,
                      Vector &topLeft, Vector &topRight, Vector &bottomLeft, Vector &bottomRight) {
    vector<Vector> points = {p1, p2, p3, p4};

    // Sort top to bottom then left to right. Done by ChatGPT. Prompt:  sort these points by their y coordinate in descending order
    sort(points.begin(), points.end(), [](const Vector &a, const Vector &b) {
        return (a.Y > b.Y) || (a.Y == b.Y && a.X < b.X);  // Reverse Y sort
    });

    // First two are top points
    if (points[0].X < points[1].X) {
        topLeft = points[0];
        topRight = points[1];
    } else {
        topLeft = points[1];
        topRight = points[0];
    }

    //last two are bottom points
    if (points[2].X < points[3].X) {
        bottomLeft = points[2];
        bottomRight = points[3];
    } else {
        bottomLeft = points[3];
        bottomRight = points[2];
    }
}

/*
 * Transforms the image
 * @param img image
 * @param matrix matrix
 * @return transformed image
 */
Img<bool> TransformImage(Img<bool> img, Matrix matrix) {

    Img<bool> transformedImage(img.Width(), img.Height());

    for (unsigned int y = 0; y < img.Height(); ++y) {
        for (unsigned int x = 0; x < img.Width(); ++x) {

            Vector v = Vector(x, y, 1);
            Vector transformed = matrix * v;

            //should always be 1 but just in case
            float x_prime = transformed.X / transformed.Z;
            float y_prime = transformed.Y / transformed.Z;

            int x_src = static_cast<int>(round(x_prime));
            int y_src = static_cast<int>(round(y_prime));

            if (x_src >= 0 && x_src < img.Width() && y_src >= 0 && y_src < img.Height()) {
                transformedImage[y][x] = img[y_src][x_src];
            }
        }
    }

    return transformedImage;
}

/*
 * Rounds the module count to the nearest valid module count
 * @param moduleCount module count
 * @return rounded module count
 */
int roundToNearesValidModuleCount(int moduleCount) {

    int closest = 21;
    int minDiff = abs(moduleCount - closest);

    for (int version = 2; version <= 40; ++version) {
        int validModuleCount = 4 * version + 17;
        int diff = abs(moduleCount - validModuleCount);

        if (diff < minDiff) {
            minDiff = diff;
            closest = validModuleCount;
        }
    }

    return closest;
}

/*
 * Calculates the module size
 * @param modulesizeX module size x
 * @param modulesizeY module size y
 * @param modulecount module count
 * @param topLeftMax top left max
 * @param bottomLeftMin bottom left min
 * @param topRightMax top right max
 * @param topRightMin top right min
 * @param topLeftMin top left min
 */
void calculateModuleSize(float &modulesizeX, float &modulesizeY, int &modulecount,
                         Vector topLeftMax, Vector bottomLeftMin, Vector topRightMax, Vector topRightMin,
                         Vector topLeftMin) {

    float dx1 = topLeftMax.X - topLeftMin.X;

    modulesizeX = dx1 / 7;

    float dy1 = topLeftMax.Y - topRightMin.Y;

    modulesizeY = dy1 / 7;

    float dx3 = topRightMax.X - bottomLeftMin.X;
    float dy3 = topLeftMax.Y - bottomLeftMin.Y;


    float modulsX = dx3 / modulesizeX;
    float modulsY = dy3 / modulesizeY;

    modulecount = (modulsX + modulsY) / 2;
    cout << "Modules x " << modulsX << endl;
    cout << "Modules y " << modulsY << endl;
    modulecount = roundToNearesValidModuleCount(modulecount);

    modulesizeX = dx3 / modulecount;
    modulesizeY = dy3 / modulecount;



}

/*
 * Checks if a module is a single module
 * @param transformed transformed image
 * @param moduleCount module count
 * @param moduleSizeX module size x
 * @param moduleSizeY module size y
 * @param ix x coordinate
 * @param iy y coordinate
 * @param topRight top right corner
 * @param bottomLeft bottom left corner
 * @return true if the module is a single module
 */
bool isSingleModule(Img<bool> &transformed, int moduleCount, float moduleSizeX, float moduleSizeY, float ix, float iy,
                    Vector topRight, Vector bottomLeft) {
    bool value = transformed[iy][ix];

    if ((transformed[iy][ix] == transformed[iy + moduleSizeY * 0.8][ix] && iy + moduleSizeY * 0.8 < topRight.Y) ||
        (transformed[iy][ix] == transformed[iy - moduleSizeY * 0.8][ix] && iy - moduleSizeY * 0.8 > bottomLeft.Y) ||
        (transformed[iy][ix] == transformed[iy][ix + moduleSizeX * 0.8] && ix + moduleSizeX * 0.8 < topRight.X) ||
        (transformed[iy][ix] == transformed[iy][ix - moduleSizeX * 0.8] && ix - moduleSizeX * 0.8 > bottomLeft.X)) {
        return false;
    }
    return true;
}
/*
 * Recenter the sampler
 * @param transformed transformed image
 * @param moduleCount module count
 * @param moduleSizeX module size x
 * @param moduleSizeY module size y
 * @param ix x coordinate
 * @param iy y coordinate
 * @param topRight top right corner
 * @param bottomLeft bottom left corner
 * @return recentered sampler
 */
Vector
recenterSampler(Img<bool> &transformed, int moduleCount, float moduleSizeX, float moduleSizeY, float ix, float iy,
                Vector topRight, Vector bottomLeft) {
    int distanceTop = 0;
    int distanceBottom = 0;
    int distanceLeft = 0;
    int distanceRight = 0;

    float dy = iy;
    float dx = ix;

    while ((transformed[iy][ix] == transformed[++dy][dx]) && dy < topRight.Y) {
        distanceTop++;
    }

    dy = iy;
    dx = ix;
    while (transformed[iy][ix] == transformed[--dy][dx] && dy > bottomLeft.Y) {
        distanceBottom++;
    }

    dy = iy;
    dx = ix;
    while (transformed[iy][ix] == transformed[dy][++dx] && dx < topRight.X) {
        distanceRight++;
    }

    dy = iy;
    dx = ix;
    while (transformed[iy][ix] == transformed[dy][--dx] && dx > bottomLeft.X) {
        distanceLeft++;
    }

    float newx = (distanceRight - distanceLeft) / 2.0f;
    float newy = (distanceTop - distanceBottom) / 2.0f;
    return Vector(newx, newy, 0);
}

/*
 * Converts a boolean image to an RGB image
 * @param img boolean image
 * @return RGB image
 */
Img<RGB_Pixel> boolToRgb(Img<bool> &img) {
    Img<RGB_Pixel> rgb(img.Width(), img.Height());
    for (unsigned int y = 0; y < img.Height(); ++y) {
        for (unsigned int x = 0; x < img.Width(); ++x) {
            rgb[y][x] = img[y][x] ? RGB_Pixel(255, 255, 255) : RGB_Pixel(0, 0, 0);
        }
    }
    return rgb;
}

/*
 * Crops the image to the QR code
 * @param rgbtransformed RGB transformed image for Debugging
 * @param transformed transformed image
 * @param moduleCount module count
 * @param moduleSizeX module size x
 * @param moduleSizeY module size y
 * @param topLeft top left corner
 * @param bottomLeft bottom left corner
 * @param topRight top right corner
 * @return cropped QR code
 */
Img<bool> cropToQRCode(Img<RGB_Pixel> &rgbtransformed, Img<bool> &transformed, int moduleCount, float moduleSizeX,
                       float moduleSizeY, Vector topLeft,
                       Vector bottomLeft, Vector topRight) {

    //if the module size is too small, increase the module count.
    //can only be done once for now
    //please dont hang me for using goto
    bool doOnce = false;
    resetSamplerWithNewModuleCount:
    if (doOnce) {
        moduleSizeX = (topRight.X - topLeft.X) / moduleCount;
        moduleSizeY = (topLeft.Y - bottomLeft.Y) / moduleCount;
    }

    //NOT SETTING THE VALUES TOOK WAY TO LONG TO DIAGNOSE. IM NOT CRYING YOU ARE
    Img<bool> croppedQRCode(moduleCount, moduleCount);
    for (unsigned int y = 0; y < moduleCount; ++y) {
        for (unsigned int x = 0; x < moduleCount; ++x) {
            croppedQRCode[y][x] = false;
        }
    }

    //adjusted so it starts in the middle of the first module
    float baseX = topLeft.X + moduleSizeX / 2;
    float baseY = topLeft.Y - moduleSizeY / 2;

    Vector adjustment(0, 0, 0);
    Vector previousInitialAdjustment(0, 0, 0);
    vector<Vector> vis; //for debugging
    bool lastSample = transformed[baseY][baseX];
    bool lastRowStartSample = lastSample;

    for (unsigned int y = 0; y < moduleCount; ++y) {

        //Adjustment for the first line
        int ix2 = baseX + adjustment.X;
        int iy2 = baseY - (moduleSizeY) * y + adjustment.Y;
        if (transformed[iy2][ix2]) {
            float distanceLeft = 0;
            float dy = iy2;
            float dx = ix2;
            while (transformed[dy][--dx]) {
                distanceLeft++;
                if (distanceLeft > moduleSizeX) {
                    distanceLeft /= 2;
                    break;
                }
            }
            float distanceRight = moduleSizeX - distanceLeft;
            adjustment.X += (distanceRight - distanceLeft) / 2.0f;
            previousInitialAdjustment = adjustment;
            lastRowStartSample = true;
        } else {
            adjustment.X = previousInitialAdjustment.X;
        }

        for (unsigned int x = 0; x < moduleCount; ++x) {

            int ix = baseX + (moduleSizeX) * x + adjustment.X;
            int iy = baseY - (moduleSizeY) * y + adjustment.Y;

            if (x == moduleCount - 1 && y == 0) { //if previous modulecount ends up being too short, adjust it onc
                float dist1 = topRight.X - ix;
                if (dist1 > moduleSizeX * 2) {
                    moduleCount += 4;
                    if (!doOnce) {
                        doOnce = true;
                        goto resetSamplerWithNewModuleCount;
                    }

                }
            }

            //if the sample is a single module, recenter the sampler
            if (isSingleModule(transformed, moduleCount, moduleSizeX, moduleSizeY, ix, iy, topRight, bottomLeft)) {

                adjustment += recenterSampler(transformed, moduleCount, moduleSizeX, moduleSizeY, ix, iy, topRight,
                                              bottomLeft);
                //Debug
                rgbtransformed[iy + 1][ix] = RGB_Pixel(0, 255, 0);
                rgbtransformed[iy - 1][ix] = RGB_Pixel(0, 255, 0);
                rgbtransformed[iy][ix + 1] = RGB_Pixel(0, 255, 0);
                rgbtransformed[iy][ix - 1] = RGB_Pixel(0, 255, 0);

            } else if (lastSample != transformed[iy][ix]) {
                float distanceLeft = 0;
                float dy = iy;
                float dx = ix;
                while (transformed[iy][ix] == transformed[dy][--dx]) {
                    distanceLeft++;
                    if (distanceLeft > moduleSizeX) {
                        distanceLeft /= 2;
                        break;
                    }

                }
                float distanceRight = moduleSizeX - distanceLeft;
                adjustment.X += (distanceRight - distanceLeft) / 2.0f;
                lastSample = transformed[iy][ix];
            }

            rgbtransformed[iy][ix] = RGB_Pixel(255, 0, 0);
            croppedQRCode[moduleCount - y - 1][x] = transformed[iy][ix];

        }
        lastSample = transformed[baseY][baseX];
        adjustment.X = 0;

    }

    return croppedQRCode;
}

/*
 * Corrects the corners of the QR code. sometimes due to round corners the corners are not exactly on the pattern
 *  this creeps them in closer to be on the pattern
 *  @param transformed transformed image
 *  @param topLeft top left corner
 *  @param topRight top right corner
 *  @param bottomLeft bottom left corner
 *  @param bottomRight bottom right corner
 */
void correctToPointOnPattern(Img<bool> transformed, Vector &topLeft, Vector &topRight, Vector &bottomLeft,
                             Vector &bottomRight) {

    while (!transformed[topLeft.Y][topLeft.X]) {
        topLeft.Y--;
        topLeft.X++;
    }
    while (!transformed[topRight.Y][topRight.X]) {
        topRight.Y--;
        topRight.X--;
    }
    while (!transformed[bottomLeft.Y][bottomLeft.X]) {
        bottomLeft.Y++;
        bottomLeft.X++;
    }
    while (!transformed[bottomRight.Y][bottomRight.X]) {
        bottomRight.Y++;
        bottomRight.X--;
    }
}

/*
 * BFS flood fill code from ChatGPT prompt: How do you implement flood fill algorithm in c++ on a binary image
 * fills the area corresponding to the point provided
 * determines the bounds of the area
 * @param img image
 * @param debug debug image
 * @param point point
 * @param minX min x
 * @param maxX max x
 * @param minY min y
 * @param maxY max y
 *
 */
void floodFill(Img<bool> &img, Img<RGB_Pixel> &debug, Vector point, int &minX, int &maxX, int &minY, int &maxY) {
    int width = img.Width();
    int height = img.Height();

    // Directions: right, left, down, up
    vector<Vector> directions = {Vector(1, 0, 0), Vector(-1, 0, 0), Vector(0, 1, 0), Vector(0, -1, 0)};

    // Visited matrix
    vector<vector<bool>> visited(height, std::vector<bool>(width, false));

    // BFS queue
    queue<Vector> q;
    q.push(point);
    visited[point.Y][point.X] = true;


    minX = maxX = point.X;
    minY = maxY = point.Y;

    while (!q.empty()) {
        Vector current = q.front();
        q.pop();

        // Update bounds
        minX = min(minX, (int) current.X);
        maxX = max(maxX, (int) current.X);
        minY = min(minY, (int) current.Y);
        maxY = max(maxY, (int) current.Y);

        // Expand in all 4 directions
        for (Vector dir: directions) {
            Vector next = Vector(current.X + dir.X, current.Y + dir.Y, 0);

            // Bounds check
            if (next.X < 0 || next.Y < 0 || next.X >= width || next.Y >= height) continue;

            // Check if it's part of the pattern and not visited
            if (img[next.Y][next.X] && !visited[next.Y][next.X]) {
                visited[next.Y][next.X] = true;

                q.push(next);
            }
        }
    }
}

/*
 * Prints the QR code to the console
 * @param croppedQRCode cropped QR code
 */
void printQRCode(Img<bool> &croppedQRCode) {
    for (unsigned int y = 0; y < croppedQRCode.Height(); ++y) {
        for (unsigned int x = 0; x < croppedQRCode.Width(); ++x) {
            cout << (croppedQRCode[y][x] ? "1 " : "0 ");
        }
        cout << endl;
    }
}

/*
 * Crops the image around the QR code where its expected to be
 * @param img image
 * @param qrCodeL vector of Object properties representing the bounding boxes of the finder patterns
 * @return cropped image
 */
Img<bool> cropAround(Img<bool> &img, vector<ObjectProperties> qrCodeL) {


    int minX = 1000000;
    int minY = 1000000;
    int maxX = 0;
    int maxY = 0;

    for (ObjectProperties qrCode: qrCodeL) {
        minX = min(minX, (int) qrCode.x_min);
        minY = min(minY, (int) qrCode.y_min);
        maxX = max(maxX, (int) qrCode.x_max);
        maxY = max(maxY, (int) qrCode.y_max);
    }
    double expansionX = (qrCodeL[0].x_max - qrCodeL[0].x_min) * 1.5;
    double expansionY = (qrCodeL[0].y_max - qrCodeL[0].y_min) * 1.5;


    minX = max(minX - expansionX, 0.0);
    minY = max(minY - expansionY, 0.0);


    maxX = min(maxX + expansionX, (double) CurrentImageWidth);
    maxY = min(maxY + expansionY, (double) CurrentImageHeight);

    Img<bool> croppedImg(maxX - minX, maxY - minY);
    for (int y = minY; y < maxY; y++) {
        for (int x = minX; x < maxX; x++) {
            croppedImg[y - minY][x - minX] = img[y][x];
        }

    }
    return croppedImg;
}

int main(int argc, char *argv[]) {

    bool writeUC = true;
    bool writeMedian = true;
    bool writeBool = true;
    bool writeLabelbild = true;
    bool writeEdges = true;
    bool writeHough = true;
    bool writeHoughTest = true;
    bool writePotentialFinderPatterns = true;
    bool writeQRCodePosition = true;

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <file1> <file2> ..." << endl;
        return 1;
    }

    // Store file paths from command-line arguments
    vector<string> files3;
    for (int i = 1; i < argc; ++i) {
        files3.push_back(argv[i]);
    }


//    string files[] = {
////            "C:\\Users\\quint\\Documents\\Studium\\HSOS\\QRScanner\\test\\test",
//            //               "C:\\Users\\quint\\Documents\\Studium\\HSOS\\QRScanner\\test2\\test2",
////                      "C:\\Users\\quint\\Documents\\Studium\\HSOS\\QRScanner\\test_r\\test_r",
//            "C:\\Users\\quint\\Documents\\Studium\\HSOS\\QRScanner\\ffb\\Untitled",
//            "C:\\Users\\quint\\Documents\\Studium\\HSOS\\QRScanner\\ffb_rotated\\ffb_r"
//    };
//
//    string files3[] = {
//            "E:\\Unity\\UnityProjects\\QRScanner\\test\\test",
////            "E:\\Unity\\UnityProjects\\QRScanner\\test2\\test2",
////            "E:\\Unity\\UnityProjects\\QRScanner\\test3\\test3",
////            "E:\\Unity\\UnityProjects\\QRScanner\\test4\\test4",
//            "E:\\Unity\\UnityProjects\\QRScanner\\ffb\\Untitled",
//            "E:\\Unity\\UnityProjects\\QRScanner\\ffb_rotated\\ffb_r"
//    };

    string t;

    for (string &filename: files3) {

        Img<bool> Binaerbild;
        Img<unsigned char> median;
        Img<unsigned char> uc;
        Img<RGB_Pixel> rgb;

        //PREPROCESSING
        {
            // read image
            filepath = filename.substr(0, filename.find_last_of("\\"));
            try {
                string fileWExtension = filename + ".bmp";
                BmpRead(fileWExtension.c_str()) >> rgb;
                cout << "Lese Datei: " << filename << endl;
            }
            catch (const char *s) {
                cerr << "Fehler beim Lesen von " << filename << ": " << s << endl;
                return -1;
            }

            CurrentImageWidth = rgb.Width();
            CurrentImageHeight = rgb.Height();

            //helper image for debugging
            vis = Img<RGB_Pixel>(CurrentImageWidth, CurrentImageHeight);

           // convert to greyscale
            uc = greyscale(rgb);
            if (writeUC) {
                try {
                    t = filename + "_uc.bmp";
                    BmpWrite(t.c_str(), uc);
                    cout << "Schreibe " << t << endl;
                } catch (const char *s) {
                    cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                    return -1;
                }
            }

            // blur image
            median = medianBlur(uc);
            if (writeMedian) {
                try {
                    t = filename + "_median.bmp";
                    BmpWrite(t.c_str(), median);
                    cout << "Schreibe " << t << endl;
                } catch (const char *s) {
                    cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                    return -1;
                }
            }

            // binarize image
            Binaerbild = binarize(median);

            //Invert image
            for (unsigned int y = 0; y < CurrentImageHeight; y++) {
                for (unsigned int x = 0; x < CurrentImageWidth; x++) {
                    bool &p = Binaerbild[y][x];
                    p = not p;
                }
            }
            if (writeBool) {
                try {
                    t = filename + "_bool.bmp";
                    BmpWrite(t.c_str(), Binaerbild);
                    cout << "Schreibe " << t << endl;
                } catch (const char *s) {
                    cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                    return -1;
                }
            }

        }

        //LABELLING adapted from lecture
        Img<unsigned int> Labelbild;
        {
            const unsigned int H(40), B(500);
            vector<unsigned int> ReferenzWerte;
            for (unsigned int i = 0; i < B; i++) {
                ReferenzWerte.push_back(i);
            }
            vector<RGB_Pixel> ReferenzFarben = Values2ColorVector<unsigned int>(ReferenzWerte);
            Img<RGB_Pixel> Referenzbild(B, H);
            for (unsigned int y = 0; y < H; y++) {
                for (unsigned int x = 0; x < B; x++) {
                    Referenzbild[y][x] = ReferenzFarben[x];
                }
            }
            string referenzbild;
            try {
                referenzbild = "Referenzbild.bmp";
                BmpWrite(referenzbild.c_str(), Referenzbild);
                cout << "Schreibe " << referenzbild << endl;
            } catch (const char *s) {
                cerr << "Fehler beim Schreiben von " << referenzbild << ": " << strerror(errno) << endl;
                return -1;
            }

            vector<pair<int, int> > Antastpunkte;
            vector<unsigned int> Objektgroessen;
            int Objekte = Labelling(Labelbild, Antastpunkte, Objektgroessen, 8, Binaerbild);
            // Fehlebehandlung
            if (Objekte < 0) {
                cerr << "Fehler beim Labelling" << endl;
                return -1;
            } else if (Objekte == 0) {
                cerr << "Keine Objekte gefunden" << endl;
                return -1;
            }
            unsigned int num_objects = Objektgroessen.size();

            // Labelbild mit verschiedenen Farben fuer die Objekte erzeugen und ausgeben
            vector<RGB_Pixel> Farben = create_LabelColors(num_objects);
            Img<RGB_Pixel> LabelAnzeige = Labelimage_to_RGB(Labelbild, Farben);

            for (unsigned int i = 0; i < Objektgroessen.size(); i++) {
                // Antastpunkte schwarz einzeichnen
                LabelAnzeige[Antastpunkte[i].second][Antastpunkte[i].first] = RGB_Pixel(0, 0, 0);
            }

            if (writeLabelbild)
                try {
                    t = filename + "_Labelbild.bmp";
                    BmpWrite(t.c_str(), LabelAnzeige);
                    cout << "Schreibe " << t << endl;
                } catch (const char *s) {
                    cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                    return -1;
                }
        }

        //FIND POTENTIAL FINDER PATTERNS AND QR CODE POSITION
        vector<ObjectProperties> qrCodeL; // L-Shaped patterns
        Img<RGB_Pixel> QRCode;
        {
            vector<ObjectProperties> obj_props = getObjectProperties(Labelbild);
            vector<ObjectProperties> potentialFinderPatterns = getPotentialFinderPatterns(obj_props);
            Img<RGB_Pixel> potentialFinderPatternsVis = visualizeObjProps(potentialFinderPatterns);

            if (writePotentialFinderPatterns) {
                try {
                    t = filename + "_PotentialFinderPatterns.bmp";
                    BmpWrite(t.c_str(), potentialFinderPatternsVis);
                    cout << "Schreibe " << t << endl;
                } catch (const char *s) {
                    cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                    return -1;
                }
            }


            qrCodeL = FindLShapedPatterns(potentialFinderPatterns);
            QRCode = visualizeObjProps(qrCodeL);


            if (writeQRCodePosition) {
                try {
                    t = filename + "_QRCodePosition.bmp";
                    BmpWrite(t.c_str(), QRCode);
                    cout << "Schreibe " << t << endl;
                } catch (const char *s) {
                    cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                    return -1;
                }
            }

            if (qrCodeL.size() != 3) {
                cerr << "Kein QR-Code gefunden" << "L-Shaped patterns:" << floor(qrCodeL.size() / 3) << endl;
                return -1;
            }
        }

        //ROTATION
        Img<bool> img(CurrentImageWidth, CurrentImageHeight); // rotiertes Bild
        {
            float angle = IsRotated(qrCodeL[0], qrCodeL[1], qrCodeL[2]);
            cout << "Angle: " << angle << endl;


            double R[3][3];           // Rotationsmatrix
            double rotVect[3];        // Rotationsvektor
            double distCoeffs[5] = {0, 0, 0, 0, 0}; // Verzerrungskoeffizienten
            double intrinsic_d[3][3];      // Intrinsische Kameraparameter
            intrinsic_d[0][0] = 1.0;
            intrinsic_d[0][1] = 0.0;
            intrinsic_d[0][2] = 0.0;
            intrinsic_d[1][0] = 0.0;
            intrinsic_d[1][1] = 1.0;
            intrinsic_d[1][2] = 0.0;
            intrinsic_d[2][0] = 0.0;
            intrinsic_d[2][1] = 0.0;
            intrinsic_d[2][2] = 1.0;


            rotVect[0] = 0;
            rotVect[1] = 0;
            rotVect[2] = angle;
            RotMat_from_Rodriguez(R, rotVect);

            UndistoreImage(img, intrinsic_d, Binaerbild, intrinsic_d, distCoeffs, rotVect);

            try {
                t = filename + "_rotated.bmp";
                BmpWrite(t.c_str(), img);
                cout << "Schreibe entzerrtes Bild: " << t << endl;
            }
            catch (const char *s) {
                cerr << "Fehler beim Schreiben von " << filename << ": " << s << endl;
                return -1;
            }
        }

        //SECOND LABELLING (i was to deep, this was not necessary and couldve been solved i na better way)
        //TODO rotate the Properties instead of the image. redoing the process of finding the finder patterns is not necessary

        vector<ObjectProperties> potentialFinderPatterns;
        {
            Img<unsigned int> Labelbild_rotated;
            vector<pair<int, int> > Antastpunkte_rotated;
            vector<unsigned int> Objektgroessen_rotated;
            int Objekte_rotated = Labelling(Labelbild_rotated, Antastpunkte_rotated, Objektgroessen_rotated, 8,
                                            img);
            // Fehlebehandlung
            if (Objekte_rotated < 0) {
                cerr << "Fehler beim Labelling" << endl;
                return -1;
            } else if (Objekte_rotated == 0) {
                cerr << "Keine Objekte gefunden" << endl;
                return -1;
            }
            unsigned int num_objects_rotated = Objektgroessen_rotated.size();


            // Labelbild mit verschiedenen Farben fuer die Objekte erzeugen und ausgeben
            vector<RGB_Pixel> Farben_rotated = create_LabelColors(num_objects_rotated);
            Img<RGB_Pixel> LabelAnzeige_rotated = Labelimage_to_RGB(Labelbild_rotated, Farben_rotated);
            for (unsigned int i = 0; i < Objektgroessen_rotated.size(); i++) { // Antastpunkte schwarz einzeichnen
                LabelAnzeige_rotated[Antastpunkte_rotated[i].second][Antastpunkte_rotated[i].first] = RGB_Pixel(0, 0,
                                                                                                                0);
            }
            try {
                t = filename + "_Labelbild_rotated.bmp";
                BmpWrite(t.c_str(), LabelAnzeige_rotated);
                cout << "Schreibe " << t << endl;
            } catch (const char *s) {
                cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                return -1;
            }

            vector<ObjectProperties> obj_props = getObjectProperties(Labelbild_rotated);
            potentialFinderPatterns = getPotentialFinderPatterns(obj_props);
            Img<RGB_Pixel> potentialFinderPatternsVis = visualizeObjProps(potentialFinderPatterns);
            try {
                t = filename + "_PotentialFinderPatterns_rotated.bmp";
                BmpWrite(t.c_str(), potentialFinderPatternsVis);
                cout << "Schreibe " << t << endl;
            } catch (const char *s) {
                cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
                return -1;
            }
        }


        qrCodeL = FindLShapedPatterns(potentialFinderPatterns);


        ObjectProperties o = EstimtateFourthCorner(qrCodeL);
        qrCodeL.push_back(o);


        //Debug
        QRCode = visualizeObjProps(qrCodeL);
        try {
            t = filename + "_QRCodePosition_rotated.bmp";
            BmpWrite(t.c_str(), QRCode);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }
        //idea was to crop the image around the QR code, but this was not done in the end to keep it simpler
        Img<bool> croppedBinary = cropAround(img, qrCodeL);
        try {
            t = filename + "_croppedBinary.bmp";
            BmpWrite(t.c_str(), croppedBinary);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }

        Img<bool> edges_rotated = edgeDetection(img);
        try {
            t = filename + "_edges_rotated.bmp";
            BmpWrite(t.c_str(), edges_rotated);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }

        //Hough Transform. due to the geometrical aproach this couldve been left out but was  a great exercise
        vector<Vector> corners = HoughTransform(edges_rotated, qrCodeL);
        if (corners.size() != 4) {
            cerr << "hough transform failed" << endl;
            continue;
        }


        Vector topLeft, topRight, bottomLeft, bottomRight;
        DetermineCorners(corners[0], corners[1], corners[2], corners[3], topLeft, topRight,
                         bottomLeft, bottomRight);
        vector<Vector> assignedCorners = {topLeft, topRight, bottomLeft,
                                          bottomRight}; //Assigned in order TL, TR, BL, BR


        vector<Vector> correctedCorners = {
                topLeft,
                topRight,
                Vector(topLeft.X, bottomLeft.Y, 0),
                Vector(topRight.X, bottomLeft.Y, 0)
        };

        //Debug
        Visualize(correctedCorners, "correctedCorners", Vector(0, 255, 0));
        Visualize(corners, "corners", Vector(0, 0, 255));


        //Perspective Transformation
        Eigen::Matrix3d perspectiveMatrix = computeHomography(assignedCorners, correctedCorners);
        Matrix m = EigneMatrix3dToMatrix(perspectiveMatrix);
        m.invert();

        Img<bool> transformed = TransformImage(img, m);
        try {
            t = filename + "_transformed.bmp";
            BmpWrite(t.c_str(), transformed);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }


        correctToPointOnPattern(transformed, topLeft, topRight, bottomLeft,
                                bottomRight); //move corners to the closest point on the pattern
        vector<Vector> cornersInPattern = {topLeft, topRight, bottomLeft, bottomRight};
        Visualize(cornersInPattern, "closestCorners", Vector(255, 255, 0));


        //Flood Fill to determine acurate bounds
        int minX, maxX, minY, maxY; //Bounds of the finder patterns. the lower left and top right corner being the min and max values respectively
        floodFill(transformed, vis, topLeft, minX, maxX, minY, maxY);
        Vector topLeftmin = Vector(minX, minY, 0);
        Vector topLeftMax = Vector(maxX, maxY, 0);
        floodFill(transformed, vis, topRight, minX, maxX, minY, maxY);
        Vector topRightmin = Vector(minX, minY, 0);
        Vector topRightMax = Vector(maxX, maxY, 0);
        floodFill(transformed, vis, bottomLeft, minX, maxX, minY, maxY);
        Vector bottomLeftmin = Vector(minX, minY, 0);
        Vector bottomLeftMax = Vector(maxX, maxY, 0);

        Visualize({topLeftmin, topLeftMax, topRightmin, topRightMax, bottomLeftMax, bottomLeftmin}, "topLeftFloodFill",
                  Vector(255, 0, 0));


        float moduleSizeX;
        float moduleSizeY;
        int moduleCount;
        calculateModuleSize(moduleSizeX, moduleSizeY, moduleCount, topLeftMax, bottomLeftmin, topRightMax, topRightmin,
                            topLeftmin);

        cout << "Module Count: " << moduleCount << endl;
        cout << "Module Size X: " << moduleSizeX << endl;
        cout << "Module Size Y: " << moduleSizeY << endl;


        Img<RGB_Pixel> rgbTransformed = boolToRgb(transformed); //To Draw debug info

        Img<bool> croppedQRCode = cropToQRCode(rgbTransformed, transformed, moduleCount, moduleSizeX, moduleSizeY,
                                               Vector(topLeftmin.X, topRightMax.Y, 0),
                                               Vector(bottomLeftmin.X, bottomLeftmin.Y, 0),
                                               Vector(topRightMax.X, topRightMax.Y, 0));


        //Invert the produced image
        for (unsigned int y = 0; y < CurrentImageHeight; y++) {
            for (unsigned int x = 0; x < CurrentImageWidth; x++) {
                bool &p = croppedQRCode[y][x];
                p = not p;
            }
        }
        printQRCode(croppedQRCode);

        try {
            t = filename + "_croppedQRCode.bmp";
            BmpWrite(t.c_str(), croppedQRCode);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }
        //Debug


        try {
            t = filename + "test.bmp";
            BmpWrite(t.c_str(), rgbTransformed);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }
    }
    return 0;

}




