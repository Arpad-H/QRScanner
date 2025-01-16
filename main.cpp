
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

#define K 5                 // Startwert fuer die Suche der Grundfrequenz
#define SE_GROESSE 3
#define INVERTED true
#define PI 3.14159265358979323846
#define DP  4           // Genauigkeit fuer double-Ausgaben
#define DW (DP + 6)     // Feldbreite fuer double-Ausgaben

struct ObjectProperties {
    Vector touch_point;
    unsigned int label;
    unsigned int x_min, x_max, y_min, y_max;
    unsigned int area;
    Vector center;
    RGB_Pixel color;
};
int CurrentImageWidth = 0;
int CurrentImageHeight = 0;
int Modules = 21; //Qr code version 1 as default

void CalculateModulePixelSize(float &sizeX, float &sizeY, const vector<ObjectProperties> &obj_props) {
    float avgWidth = 0;
    float avgHeight = 0;
    for (const auto &obj: obj_props) {

        float width = abs(obj.center.X - obj.touch_point.X) * 2;
        float height = abs(obj.center.Y - obj.touch_point.Y) * 2;
        avgWidth += width;
        avgHeight += height;

    }
    avgWidth /= obj_props.size();
    avgHeight /= obj_props.size();

    sizeX = avgWidth / 7;
    sizeY = avgHeight / 7;

}


ObjectProperties VectorToProperties(Vector v) {

    ObjectProperties obj;
    //obj.label = label;

    obj.center = v;
    obj.x_min = v.X - 10;
    obj.x_max = v.X + 10;
    obj.y_min = v.Y - 10;
    obj.y_max = v.Y + 10;

    return obj;
}

Img<RGB_Pixel>
HighlightPotentialPatterns(const vector<ObjectProperties> &obj_props) {

    Img<RGB_Pixel> QRHighlited(CurrentImageWidth, CurrentImageHeight);

    for (const auto &obj: obj_props) {


        for (unsigned int y = obj.y_min; y <= obj.y_max; y++) {
            for (unsigned int x = obj.x_min; x <= obj.x_max; x++) {
                if (y == obj.y_min || y == obj.y_max || x == obj.x_min || x == obj.x_max) {
                    QRHighlited[y][x] = RGB_Pixel(255, 0, 0);
                    QRHighlited[obj.center.Y][obj.center.X] = RGB_Pixel(0, 255, 0);
                }
//                    QRHighlited[y][x] = RGB_Pixel(255, 0, 0);
            }

        }

    }


    return QRHighlited;
}

void Visualize(const vector<Vector> &points) {
//    // Img<unsigned int> empty(CurrentImageWidth, CurrentImageHeight);
//    Img<RGB_Pixel> vis(CurrentImageWidth, CurrentImageHeight);
//    //cout << "obj_props.size(): " << obj_props.size() << endl;
//    string filename;
//    for (const auto &obj: obj_props) {
//
//       // cout << "obj.x_min: " << obj.x_min << " obj.x_max: " << obj.x_max << " obj.y_min: " << obj.y_min
//       //      << " obj.y_max: " << obj.y_max << endl;
//        for (unsigned int y = obj.y_min; y <= obj.y_max; y++) {
//            for (unsigned int x = obj.x_min; x <= obj.x_max; x++) {
//                if (y == obj.y_min || y == obj.y_max || x == obj.x_min || x == obj.x_max) {
//                    vis[y][x] = obj.color;
//                    vis[obj.center.Y][obj.center.X] = RGB_Pixel(255, 0, 0);
//                }
////                    QRHighlited[y][x] = RGB_Pixel(255, 0, 0);
//            }
//
//        }

//    }
//    try {
//        filename = "E:\\Unity\\UnityProjects\\QRScanner\\viz.bmp";
//        BmpWrite(filename.c_str(), vis);
//        cout << "Schreibe viz Bild: " << filename << endl;
//    }
//    catch (const char *s) {
//        cerr << "Fehler beim Schreiben von " << filename << ": " << s << endl;
//
//    }
}

void Print_RotMat(double R[3][3]) {
    cout << "Rotationsmatrix (berechnet aus rx, ry, rz):" << endl;
    cout << setw(DW) << R[0][0] << setw(DW) << R[0][1] << setw(DW) << R[0][2] << endl;
    cout << setw(DW) << R[1][0] << setw(DW) << R[1][1] << setw(DW) << R[1][2] << endl;
    cout << setw(DW) << R[2][0] << setw(DW) << R[2][1] << setw(DW) << R[2][2] << endl;
    cout << endl;
}

bool IsPotentialFinderPattern(const ObjectProperties &obj, const unsigned int MIN_AREA, const unsigned int MAX_AREA) {
    unsigned int width = obj.x_max - obj.x_min + 1;
    unsigned int height = obj.y_max - obj.y_min + 1;

    // Check square aspect ratio
    if (abs((int) width - (int) height) > max(width, height) * 0.2) {
        return false;
    }

//    // Check size constraints
    if (obj.area < MIN_AREA || obj.area > MAX_AREA) {
        return false;
    }

    return true;
}

Img<RGB_Pixel> Cropped(const Img<bool> &labelled, vector<ObjectProperties> props, int dim) {

    //Get bounds of qr code
    int x_min = 10000;
    int y_min = 10000;
    int x_max, y_max = 0;
    for (const auto &obj: props) {
        if (IsPotentialFinderPattern(obj, 100, 1000)) {

            if (obj.x_min < x_min) {
                x_min = obj.x_min;
            }
            if (obj.x_max > x_max) {
                x_max = obj.x_max;
            }
            if (obj.y_min < y_min) {
                y_min = obj.y_min;
            }
            if (obj.y_max > y_max) {
                y_max = obj.y_max;
            }
        }
    }
    int width = x_max - x_min;
    int height = y_max - y_min;
   // cout << "width: " << width << " height: " << height << endl;
    Img<RGB_Pixel> cropped(dim, dim);

//    cropped[y_max][x_min] = RGB_Pixel(255, 255, 255);
//    cropped[y_max][x_max] = RGB_Pixel(255, 255, 0);
//    cropped[y_min][x_min] = RGB_Pixel(255, 0, 255);
//    cropped[y_min][x_max] = RGB_Pixel(0, 255, 0);
//    return cropped;
    int ix = 0;
    int iy = 0;
    // Calculate scaling factors
    float scaleX = static_cast<float>(width) / dim;
    float scaleY = static_cast<float>(height) / dim;
    float offset = scaleX / 2;
    for (float y = y_min; y <= y_max; y += scaleY) {
        for (float x = x_min; x <= x_max; x += scaleX) {
//            int scaledX = static_cast<int>(x * scaleX) + x_min;
//            int scaledY = static_cast<int>(y * scaleY) + y_min;
            cropped[iy][ix] = labelled[y + offset][x + offset];
            ix++;
        }
        ix = 0;
        iy++;
//    for (unsigned int y = 0; y < dim; y++) {
//        for (unsigned int x = 0; x < dim; x++) {
//            if (y < y_max && y > y_min && x > x_min && x < x_max) {
//                int scaledX = static_cast<int>(x * scaleX) + x_min;
//                int scaledY = static_cast<int>(y * scaleY) + y_min;
//                cropped[y][x] = labelled[scaledY][scaledX];
////            }

//        }
    }
//    cropped[0][0] = RGB_Pixel(255, 255, 255);

    return cropped;
}

Img<bool> Binary(const Img<RGB_Pixel> &image, unsigned char threshold) {
    const unsigned int width = image.Width();
    const unsigned int height = image.Height();
    Img<bool> binary(width, height);

    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            binary[y][x] =
                    image[y][x].Red() > threshold || image[y][x].Green() > threshold || image[y][x].Blue() > threshold;
        }
    }

    return binary;
}

vector<ObjectProperties> GetObjectProperties(const Img<unsigned int> &label_image) {
    map<unsigned int, ObjectProperties> objects;

    for (unsigned int y = 0; y < label_image.Height(); y++) {
        for (unsigned int x = 0; x < label_image.Width(); x++) {
            unsigned int label = label_image[y][x];
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

// ----------------------------------------------------------------------------------------------------
// Labelbild in RGB-Bild konvertieren unter Verwendung eines Vektors,
// der den Labelwerten RGB-Farben zuordnet
// ----------------------------------------------------------------------------------------------------
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

vector<ObjectProperties> getPotentialFinderPatterns(const vector<ObjectProperties> &obj_props) {
    vector<ObjectProperties> potentialFinderPatterns;
    for (const auto &obj: obj_props) {
        if (IsPotentialFinderPattern(obj, 100, 1000)) {
            potentialFinderPatterns.push_back(obj);
        }
    }
    return potentialFinderPatterns;
}


float Distance(ObjectProperties a, ObjectProperties b) {

    return abs((a.center - b.center).length());
}

bool IsLShape(const ObjectProperties &a, const ObjectProperties &b, const ObjectProperties &c) {


    Vector av = a.center;
    Vector bv = b.center;
    Vector cv = c.center;

// Step 1: Calculate distances between all points
    float d1 = (bv - av).length(); // Distance AB
    float d2 = (cv - av).length(); // Distance AC
    float d3 = (cv - bv).length(); // Distance BC
    // Step 2: Identify the hypotenuse
    float max_d = max(d1, max(d2, d3));
    Vector topLeft, topRight, bottomLeft;
    Vector midPoint;

    if (max_d == d1) { // AB is the hypotenuse
        midPoint = (av + bv) * 0.5f;
        if ((cv - midPoint).length() > (av - midPoint).length()) {
            topLeft = cv;
            topRight = av;
            bottomLeft = bv;
        } else {
            topLeft = cv;
            topRight = bv;
            bottomLeft = av;
        }

    } else if (max_d == d2) { // AC is the hypotenuse
        midPoint = (av + cv) * 0.5f;
        if ((bv - midPoint).length() > (av - midPoint).length()) {
            topLeft = bv;
            topRight = av;
            bottomLeft = cv;
        } else {
            topLeft = bv;
            topRight = cv;
            bottomLeft = av;
        }
    } else if (max_d == d3) { // BC is the hypotenuse
        midPoint = (bv + cv) * 0.5f;
        if ((av - midPoint).length() > (bv - midPoint).length()) {
            topLeft = av;
            topRight = bv;
            bottomLeft = cv;
        } else {
            topLeft = av;
            topRight = cv;
            bottomLeft = bv;
        }
    }
    float dot = ((topLeft - topRight).normalize()).dot((topLeft - bottomLeft).normalize());
  //  cout << "dot: " << dot << endl;
    if (dot - 0.1 < 0 && dot + 0.1 > 0) {
        return true;
    }
    return false;

}

vector<ObjectProperties> FindOverlappingProps(const vector<ObjectProperties> &obj_props) {
    vector<ObjectProperties> overlappingProps;
    float max_deviation = 3;
    for (int i = 0; i < obj_props.size(); i++) {
        for (int j = i + 1; j < obj_props.size(); j++) {
            //only look for centers overlapping with a certain max deviation
            if (Distance(obj_props[i], obj_props[j]) < max_deviation) {
                overlappingProps.push_back(obj_props[i]);
                overlappingProps.push_back(obj_props[j]);
            }
        }
    }
    return overlappingProps;
}

vector<ObjectProperties> FindLShapedPatterns(const vector<ObjectProperties> &obj_props) {


//    vector<ObjectProperties> bestPatterns;
//    for (int i = 0; i < obj_props.size(); i++) {
//        for (int j = i + 1; j < obj_props.size(); j++) {
//            for (int k = j + 1; k < obj_props.size(); k++) {
//                if (IsLShape(obj_props[i], obj_props[j], obj_props[k])) {
//                    bestPatterns = {obj_props[i], obj_props[j], obj_props[k]};
//                    return bestPatterns;
//                }
//            }
//        }
//    }
//    return {};
    vector<ObjectProperties> bestPatterns;
    for (int i = 0; i < obj_props.size(); i++) {
        for (int j = i + 1; j < obj_props.size(); j++) {
            for (int k = j + 1; k < obj_props.size(); k++) {
                if (IsLShape(obj_props[i], obj_props[j], obj_props[k])) {
                    bestPatterns = {obj_props[i], obj_props[j], obj_props[k]};
                    return bestPatterns;
                }
            }
        }
    }
    return {};
}

float IsRotated(const ObjectProperties &a, const ObjectProperties &b, const ObjectProperties &c) {
    Vector av = a.center;
    Vector bv = b.center;
    Vector cv = c.center;

    Vector ab = bv - av;
    Vector ac = cv - av;
    Vector bc = cv - bv;

// Step 1: Calculate distances between all points
    float d1 = (bv - av).length(); // Distance AB
    float d2 = (cv - av).length(); // Distance AC
    float d3 = (cv - bv).length(); // Distance BC
    // Step 2: Identify the hypotenuse
    float max_d = max(d1, max(d2, d3));
    Vector topLeft, topRight, bottomLeft;
    Vector midPoint;

    if (max_d == d1) { // AB is the hypotenuse
        midPoint = av + (ab * 0.5f);
        Vector cm = cv - midPoint;
        Matrix mrot;
        // cout << "cm: " << cm.X << " " << cm.Y << " " << cm.Z << endl;
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
    // Step 3: Calculate rotation angle
    Vector horizontal = Vector(1, 0, 0);
    Vector topEdge = topRight - topLeft;
    topEdge.normalize();
   // cout << "topEdge: " << topEdge.X << " " << topEdge.Y << " " << topEdge.Z << endl;

    float angle = acos(horizontal.dot(topEdge)); // Angle with x-axis
    if (topEdge.Y < 0) {
        angle = -angle; // Adjust for negative rotation
    }
    return angle;
}


// ----------------------------------------------------------------------------------------------------
// Vektor zufaelliger RGB-Farben fuer die Label eines Labelbilds erzeugen
// ----------------------------------------------------------------------------------------------------
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


// ----------------------------------------------------------------------------------------------------
// Vektor aufsteigender RGB-Farben fuer einen Vektor mit Merkmalswerten erzeugen
// ----------------------------------------------------------------------------------------------------
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
    vector<RGB_Pixel> sort_colors; // enthaelt Farben in Reihenfolge der Objektgroessen
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

vector<Position> mirror_SE(const vector<Position> &ImageWindow) {
    vector<Position> MirroredImageWindow(ImageWindow.size());

    for (const auto &pos: ImageWindow) {
        MirroredImageWindow.push_back(Position{-pos.get_x(), -pos.get_y()});
    }

    return MirroredImageWindow;
}

template<typename Pixel>
Img<Pixel> closing(const Img<Pixel> &src, const vector<Position> &ImageWindow) {
    Img<Pixel> closed;

    Img<Pixel> dilated = dilate(src, ImageWindow);


    std::vector<Position> mirrored_SE = mirror_SE(ImageWindow);
    closed = erode(dilated, mirrored_SE);

    return closed;
}

// ---------------------------------------------------------------------------
// Aufgabe 5: Code erstellen
// Bilddifferenz
// ---------------------------------------------------------------------------
// Parameter:
// [in]  l : Bild links vom Operator '-'
// [in]  r : Bild rechts vom Operator '-'
// Return:
// Differenzbild
// ---------------------------------------------------------------------------
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

template<typename Pixel>
Img<Pixel> opening(const Img<Pixel> &src, const vector<Position> &ImageWindow) {
    Img<Pixel> opened;

    Img<Pixel> eroded = erode(src, ImageWindow);


    std::vector<Position> mirrored_SE = mirror_SE(ImageWindow);
    opened = dilate(eroded, mirrored_SE);

    return opened;
}

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

Img<double> greyscale(const Img<RGB_Pixel> &rgb) {
    const unsigned int width = rgb.Width();
    const unsigned int height = rgb.Height();
    Img<double> gray;

    unsigned int ld;
    unsigned int val;
    for (ld = 0, val = width; val != 0; val = val >> 1) {
        ld++;
    }
    if (static_cast<unsigned int>(1 << (ld - 1)) != width) {
        cerr << "Bildbreite ist keine Zweierpotenz, fft kann nicht verwendet werden" << endl;
        return -1;
    }
    for (ld = 0, val = height; val != 0; val = val >> 1) {
        ld++;
    }
    if (static_cast<unsigned int>(1 << (ld - 1)) != height) {
        cerr << "Bildhoehe ist keine Zweierpotenz, fft kann nicht verwendet werden" << endl;
        return -1;
    }
    gray = ConvImg<double, RGB_Pixel>(rgb);
    return gray;
}

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

//TODO FIX
Img<bool> optimal_threshold(const Img<unsigned char> &gray_image) {
    const unsigned int Width = gray_image.Width();
    const unsigned int Height = gray_image.Height();
    Img<bool> binary_image(Width, Height);

    // Histogramm berechnen
    std::vector<unsigned int> histogram(256, 0);
    for (unsigned int y = 0; y < Height; ++y) {
        for (unsigned int x = 0; x < Width; ++x) {
            ++histogram[gray_image[y][x]];
        }
    }

    // Gesamtanzahl der Pixel
    const unsigned int total_pixels = Width * Height;

    // Otsu-Methode: Optimalen Schwellenwert berechnen
    double sum = 0.0;
    for (unsigned int i = 0; i < 256; ++i) {
        sum += i * histogram[i];
    }

    double sum_background = 0.0;
    unsigned int weight_background = 0;
    unsigned int weight_foreground = 0;

    double max_variance = 0.0;
    unsigned int optimal_threshold = 0;

    for (unsigned int t = 0; t < 256; ++t) {
        weight_background += histogram[t];
        if (weight_background == 0) continue;

        weight_foreground = total_pixels - weight_background;
        if (weight_foreground == 0) break;

        sum_background += t * histogram[t];

        double mean_background = sum_background / weight_background;
        double mean_foreground = (sum - sum_background) / weight_foreground;

        double variance_between = weight_background * weight_foreground *
                                  std::pow(mean_background - mean_foreground, 2);

        if (variance_between > max_variance) {
            max_variance = variance_between;
            optimal_threshold = t;
        }
    }
//    optimal_threshold = 145;
    optimal_threshold = 145;
    cout << "Schwellwert: " << optimal_threshold << endl;
    // Binärbild erzeugen
    for (unsigned int y = 0; y < Height; ++y) {
        for (unsigned int x = 0; x < Width; ++x) {
            binary_image[y][x] = gray_image[y][x] > optimal_threshold;
        }
    }

    return binary_image;
}

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

void CalculateModuleCount(const Img<RGB_Pixel> &rotatedImage) {
    vector<ObjectProperties> obj_props = GetObjectProperties(ConvImg<unsigned int, RGB_Pixel>(rotatedImage));
    vector<ObjectProperties> potentialFinderPatterns = getPotentialFinderPatterns(obj_props);
    //Img<RGB_Pixel> potentialFinderPatternsVis = HighlightPotentialPatterns(potentialFinderPatterns, Labelbild);
//    try {
//        t = filename + "_PotentialFinderPatterns.bmp";
//        BmpWrite(t.c_str(), potentialFinderPatternsVis);
//        cout << "Schreibe " << t << endl;
//    } catch (const char *s) {
//        cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
//        return -1;
//    }

    vector<ObjectProperties> qrCode = FindOverlappingProps(potentialFinderPatterns);
    vector<ObjectProperties> qrCodeL = FindLShapedPatterns(qrCode);
    //Img<RGB_Pixel> QRCode = HighlightPotentialPatterns(qrCodeL);
    float moduleXPixels, moduleYPixels;
    CalculateModulePixelSize(moduleXPixels, moduleYPixels, qrCodeL);
cout << "ModuleXPixels: " << moduleXPixels << " ModuleYPixels: " << moduleYPixels << endl;
    Vector av = qrCodeL[0].center;
    Vector bv = qrCodeL[1].center;
    Vector cv = qrCodeL[2].center;

// Step 1: Calculate distances between all points
    float d1 = (bv - av).length(); // Distance AB
    float d2 = (cv - av).length(); // Distance AC
    float d3 = (cv - bv).length(); // Distance BC
    // Step 2: Identify the hypotenuse
    float max_d = max(d1, max(d2, d3));
    ObjectProperties topLeft, topRight, bottomLeft;
    Vector midPoint;

    if (max_d == d1) { // AB is the hypotenuse
        midPoint = (av + bv) * 0.5f;
        if ((cv - midPoint).length() > (av - midPoint).length()) {
            topLeft = qrCodeL[2];
            topRight = qrCodeL[0];
            bottomLeft = qrCodeL[1];
        } else {
            topLeft = qrCodeL[2];
            topRight = qrCodeL[1];
            bottomLeft = qrCodeL[0];
        }

    } else if (max_d == d2) { // AC is the hypotenuse
        midPoint = (av + cv) * 0.5f;
        if ((bv - midPoint).length() > (av - midPoint).length()) {
            topLeft = qrCodeL[1];
            topRight = qrCodeL[0];
            bottomLeft = qrCodeL[2];
        } else {
            topLeft = qrCodeL[1];
            topRight = qrCodeL[2];
            bottomLeft = qrCodeL[0];
        }
    } else if (max_d == d3) { // BC is the hypotenuse
        midPoint = (bv + cv) * 0.5f;
        if ((av - midPoint).length() > (bv - midPoint).length()) {
            topLeft = qrCodeL[0];
            topRight = qrCodeL[1];
            bottomLeft = qrCodeL[2];
        } else {
            topLeft = qrCodeL[0];
            topRight = qrCodeL[2];
            bottomLeft = qrCodeL[1];
        }
    }
    float dx = topRight.x_max - topLeft.x_min;
    float dy = topLeft.y_max - bottomLeft.y_min;
cout << "dx: " << dx << " dy: " << dy << endl;
    int moduleSizeX = dx /moduleXPixels;
    int moduleSizeY = dy /moduleYPixels;

    cout << "Module Size X: " << moduleSizeX << " Module Size Y: " << moduleSizeY << endl;

    Modules = (moduleSizeX + moduleSizeY) / 2;

}

int main(int argc, char *argv[]) {
    string files[] = {"E:\\Unity\\UnityProjects\\QRScanner\\test\\test",
                      "E:\\Unity\\UnityProjects\\QRScanner\\test_90\\test_90",
                      "E:\\Unity\\UnityProjects\\QRScanner\\test_r\\test_r"};
    string t;
    //string filename = "E:\\Unity\\UnityProjects\\QRScanner\\test\\test";
    //string filename = "E:\\Unity\\UnityProjects\\QRScanner\\test_90\\test_90";
    //string filename = "E:\\Unity\\UnityProjects\\QRScanner\\test_r\\test_r";
    for (string &filename: files) {


        // Bild einlesen
        Img<RGB_Pixel> rgb;
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

        // --------------------------------------------------------------------------------
        // Binaeres Quellbild erzeugen
        // --------------------------------------------------------------------------------
        const unsigned int height = rgb.Height();
        const unsigned int width = rgb.Width();


        Img<unsigned char> uc = ConvImg<unsigned char, RGB_Pixel>(rgb);
        try {
            t = filename + "_uc.bmp";
            BmpWrite(t.c_str(), uc);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }


        vector<Position> quadratisches_Bildfenster;
//    quadratisches_Bildfenster = create_round_SE(SE_GROESSE);
//    Img<unsigned char> eroded;
//    try {
//        // Erosion durchfuehren
//        eroded = opening(uc, quadratisches_Bildfenster);
//         t  = filename + "_eroded_sq.bmp";
//        BmpWrite(t.c_str(), eroded);
//        cout << "Schreibe " << t << endl;
//    } catch (const char * s) {
//        cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
//        return -1;
//    }

        Img<unsigned char> median = medianBlur(uc);
        try {
            t = filename + "_median.bmp";
            BmpWrite(t.c_str(), median);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }

        Img<bool> Binaerbild = optimal_threshold(median);
#if true == INVERTED
        // Bei Bedarf Binaerbild invertieren: Objektpixel muessen "true" sein
        for (unsigned int y = 0; y < height; y++) {
            for (unsigned int x = 0; x < width; x++) {
                bool &p = Binaerbild[y][x];
                p = not p;
            }
        }
#endif

        try {
            t = filename + "_bool.bmp";
            BmpWrite(t.c_str(), Binaerbild);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }


        Img<bool> edges = edgeDetection(Binaerbild);
        try {
            t = filename + "_edges.bmp";
            BmpWrite(t.c_str(), edges);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }
//
//    Img<double> src = greyscale(rgb);
//    //write greyscale image
//    {
//        Img<unsigned char> uc_src = ConvImg<unsigned char, double>(src, 255.0, 0.0);
//        try
//        {
//            filename = filename + "_gray.bmp";
//            BmpWrite(filename.c_str(), uc_src);
//            cout << "Schreibe Datei: " << filename << endl;
//        }
//        catch (const char * s)
//        {
//            cerr << "Fehler beim Schreiben von " << filename << ": " << s << endl;
//            return -1;
//        }
//    }
//    double th = threshold(src);
//    cout << "Schwellwert: " << th << endl;
//  //  Img<double> bin;
//    //binary(bin, th);
// --------------------------------------------------------------------------------
        // Referenzbild mit den Farben zum Einfaerben der Merkmale erzeugen
        // --------------------------------------------------------------------------------
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
            referenzbild = "E:\\Unity\\UnityProjects\\QRScanner\\Referenzbild.bmp";
            BmpWrite(referenzbild.c_str(), Referenzbild);
            cout << "Schreibe " << referenzbild << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << referenzbild << ": " << strerror(errno) << endl;
            return -1;
        }

        // ------------------------------------------------
        // Zu Aufgabe 1: Labelling des Bildes durchfuehren
        // ------------------------------------------------

        Img<unsigned int> Labelbild;
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
        cout << "Gefundene Objekte: " << num_objects << endl;


        // Labelbild mit verschiedenen Farben fuer die Objekte erzeugen und ausgeben
        vector<RGB_Pixel> Farben = create_LabelColors(num_objects);
        Img<RGB_Pixel> LabelAnzeige = Labelimage_to_RGB(Labelbild, Farben);
        for (unsigned int i = 0; i < Objektgroessen.size(); i++) { // Antastpunkte schwarz einzeichnen
            LabelAnzeige[Antastpunkte[i].second][Antastpunkte[i].first] = RGB_Pixel(0, 0, 0);
        }
        try {
            t = filename + "_Labelbild.bmp";
            BmpWrite(t.c_str(), LabelAnzeige);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }

        vector<ObjectProperties> obj_props = GetObjectProperties(Labelbild);
        vector<ObjectProperties> potentialFinderPatterns = getPotentialFinderPatterns(obj_props);
        Img<RGB_Pixel> potentialFinderPatternsVis = HighlightPotentialPatterns(potentialFinderPatterns);
        try {
            t = filename + "_PotentialFinderPatterns.bmp";
            BmpWrite(t.c_str(), potentialFinderPatternsVis);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }

        vector<ObjectProperties> qrCode = FindOverlappingProps(potentialFinderPatterns);
        vector<ObjectProperties> qrCodeL = FindLShapedPatterns(qrCode);
        Img<RGB_Pixel> QRCode = HighlightPotentialPatterns(qrCodeL);
        try {
            t = filename + "_QRCodePosition.bmp";
            BmpWrite(t.c_str(), QRCode);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }
        if (qrCodeL.size() != 3) {
            cerr << "Kein QR-Code gefunden" << endl;
            return -1;
        }
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
       // Print_RotMat(R);


        Img<RGB_Pixel> img(width, height); // entzerrtes Bild

        // Calculate_Intrinsics(intrinsic_d, img_d.Width(), img_d.Height(), distCoeffs, rotVect, intrinsic, width, height);

        UndistoreImage(img, intrinsic_d, rgb, intrinsic_d, distCoeffs, rotVect);

        // Entzerrtes Bild "img" wegspeichern
        try {
            t = filename + "_entzerrt.bmp";
            BmpWrite(t.c_str(), img);
            cout << "Schreibe entzerrtes Bild: " << t << endl;
        }
        catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << filename << ": " << s << endl;
            return -1;
        }


        CalculateModuleCount(img);
        cout << "Modules: " << Modules << endl;

        Binaerbild = optimal_threshold(median);
        Img<RGB_Pixel> cropped = Cropped(Binaerbild, qrCode, 33);
        try {
            t = filename + "_cropped.bmp";
            BmpWrite(t.c_str(), cropped);
            cout << "Schreibe " << t << endl;
        } catch (const char *s) {
            cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
            return -1;
        }

//    Img<unsigned char> cropped_bw = ConvImg<unsigned char, RGB_Pixel>(cropped);
//    Img<bool> binary_cropped = optimal_threshold(cropped_bw);
//    try {
//        t = filename + "_cropped_binary.bmp";
//        BmpWrite(t.c_str(), binary_cropped);
//        cout << "Schreibe " << t << endl;
//    } catch (const char *s) {
//        cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
//        return -1;
//    }
//    // Objektbild erzeugen, in dem die Objekte gemaess ihrer Groesse eingefaerbt sind
//    // (Einfaerbung 0:blau, Mitte:gruen, Maximum:rot)
//    vector<RGB_Pixel> GroesseFarben = Values2ColorVector<unsigned int>(Objektgroessen);
//    Img<RGB_Pixel> GroesseAnzeige = Labelimage_to_RGB(Labelbild, GroesseFarben);
//    try {
//        t = filename + "_Groesse.bmp";
//        BmpWrite(t.c_str(), GroesseAnzeige);
//        cout << "Schreibe " << t << endl;
//    } catch (const char *s) {
//        cerr << "Fehler beim Schreiben von " << t << ": " << strerror(errno) << endl;
//        return -1;
//    }


    }
    return 0;
}

