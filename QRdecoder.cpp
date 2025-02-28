//
// Created by quint on 2/11/2025.
//

#include "QRdecoder.h"

void QRdecoder::decodeQR(const string& filepath) {
    FILEPATH = filepath;
    FILENAME = filepath.substr(filepath.find_last_of("\\")+2, filepath.length()-1);
    Img<RGB_Pixel> img = readImage();
    resize(img);
    Img<unsigned char> grey = greyscale(img);
    Img<unsigned char> blur = medianBlur(grey);
   // blur = medianBlur(blur);
    Img<bool> binary = binarization(blur);
    writeImage(binary, "_binary.bmp");

    vector<Vector> finderPatterns = findFinderPatterns(binary);
    cout << "Anzahl der Finder Patterns: " << finderPatterns.size() << endl;
    visualizeVectors(finderPatterns, 2, false, 0);
}

Img<RGB_Pixel> QRdecoder::readImage() {
    Img<RGB_Pixel> rgb;
    string completePath;
    try {
completePath = FILEPATH + ".bmp";
        BmpRead(completePath.c_str()) >> rgb;
        cout << "Lese Datei: " << FILENAME << endl;
    } catch (const char *s) {
        cerr << s << endl;
    }
    imageHeight = rgb.Height();
    imageWidth = rgb.Width();
    return rgb;
}

Img<unsigned char> QRdecoder::greyscale(Img<RGB_Pixel> img) {
    Img<unsigned char> grey(img.Width(), img.Height());
    for (unsigned int y = 0; y < img.Height(); ++y) {
        for (unsigned int x = 0; x < img.Width(); ++x) {
            grey[y][x] = static_cast<unsigned char>(0.299 * img[y][x].Red() + 0.587 * img[y][x].Green() + 0.114 * img[y][x].Blue());
        }
    }
    return grey;
}

Img<unsigned char> QRdecoder::medianBlur(const Img<unsigned char> &gray) {
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
Img<bool> QRdecoder::binarization(const Img<unsigned char> &grey) {
    Img<bool> binary(grey.Width(), grey.Height());
    unsigned int threshold = optimalThreshold(grey);
    for (unsigned int y = 0; y < grey.Height(); ++y) {
        for (unsigned int x = 0; x < grey.Width(); ++x) {
            binary[y][x] = grey[y][x] > threshold;
        }
    }
    return binary;
}

unsigned int QRdecoder::optimalThreshold(const Img<unsigned char> &gray_image) {


    unsigned int T = 128; // Initial threshold
    unsigned int prev_T;

    do {
        prev_T = T;
        unsigned int sum1 = 0, sum2 = 0;
        unsigned int count1 = 0, count2 = 0;

        // Calculate means for pixels below and above threshold
        for (unsigned int y = 0; y < imageHeight; ++y) {
            for (unsigned int x = 0; x < imageWidth; ++x) {
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
    } while (T != prev_T); // Stop when threshold stabilizes
    return T;
}

void QRdecoder::writeImage(Img<bool> binary, string fileextention) {
    string completePath;
    try {
        completePath = FILEPATH + fileextention;
        BmpWrite(completePath.c_str(), binary);
        cout << "Schreibe " << completePath << endl;
    } catch (const char *s) {
        cerr << "Fehler beim Schreiben von " << completePath << ": " << s << endl;
    }
}

bool QRdecoder::is_approx_ratio(const vector<int>& lengths, const vector<double>& target_ratio, double tolerance) {
    if (lengths.size() != target_ratio.size()) return false;

    // Compute the average of the 1:1:3:1:1 segments (excluding the middle block)
    double sameRatioAvg = (lengths[0] + lengths[1] + lengths[3] + lengths[4]) / 4.0;

    // Validate the 1:1:3:1:1 ratio
    for (size_t i = 0; i < lengths.size(); i++) {
        double expectedLength = (i == 2) ? (sameRatioAvg * 3) : sameRatioAvg;  // Middle block is 3x
        double minBound = expectedLength * (1.0 - tolerance);
        double maxBound = expectedLength * (1.0 + tolerance);

        if (lengths[i] < minBound || lengths[i] > maxBound) {
            return false;
        }
    }

    return true;
}


//bool QRdecoder::detect_finder_pattern(const Img<bool> &binary_image, int y, int x) {
//    vector<double> finder_pattern_ratio = {1, 1, 3, 1, 1};
//
//    vector<int> horizontal;
//    vector<int> vertical;
//
//    // Scan horizontally
//    int current = binary_image[y][x];
//    int count = 0;
//    for (int i = x; i < binary_image.Width(); ++i) {
//        if (binary_image[y][i] == current) {
//            count++;
//        } else {
//            horizontal.push_back(count);
//            count = 1;
//            current = binary_image[y][i];
//        }
//        if (horizontal.size() == 5) break;
//    }
//    // Scan vertically
//    current = binary_image[y][x];
//    count = 0;
//    for (int i = y; i < binary_image.Height(); ++i) {
//        if (binary_image[i][x] == current) {
//            count++;
//        } else {
//            vertical.push_back(count);
//            count = 1;
//            current = binary_image[i][x];
//        }
//        if (vertical.size() == 5) break;
//    }
//
//    return is_approx_ratio(horizontal, finder_pattern_ratio) || is_approx_ratio(vertical, finder_pattern_ratio);
//}

void QRdecoder::visualizeVectors(vector<Vector> location, int thickness, bool border, int borderDistance) {
    Img<RGB_Pixel> vis(imageWidth, imageHeight);
    for (const auto &point: location) {
        vis[point.Y][point.X] = RGB_Pixel(255, 0, 255);
        vis[point.Y - 1][point.X] = RGB_Pixel(255, 0, 255);
        vis[point.Y + 1][point.X] = RGB_Pixel(255, 0, 255);
        vis[point.Y][point.X - 1] = RGB_Pixel(255, 0, 255);
        vis[point.Y][point.X + 1] = RGB_Pixel(255, 0, 255);
    }
    string filename;
    try {
        filename = FILEPATH + "_viz" + to_string(vizCounter)+ ".bmp";
        BmpWrite(filename.c_str(), vis);
        cout << "Schreibe viz Bild: " << filename << endl;
    } catch (const char *s) {
        cerr << "Fehler beim Schreiben von " << filename << ": " << s << endl;
    }
}

vector<Vector> QRdecoder::findFinderPatterns(const Img<bool> &binary_image) {
    vector<Vector> patterns;
    for (int y = 0; y < imageWidth; y += 2) {
        vector<int> lengths;
        int current = binary_image[0][y];
        int count = 0;

        // Scan horizontally in the column
        for (int x = 0; x < imageHeight; ++x) {

                if (validate_vertical_ratio(binary_image, x, y)) {
                    patterns.emplace_back(Vector(x, y, 0));
                }
            }
        }

//    for (int y = 0; y < imageHeight; y += 2) {
//            vector<int> lengths;
//            int current = binary_image[y][0];
//            int count = 0;
//
//            // Scan horizontally in the row
//            for (int x = 0; x < imageWidth; ++x) {
//                if (binary_image[y][x] == current) {
//                    count++;
//                } else {
//                    lengths.push_back(count);
//                    count = 1;
//                    current = binary_image[y][x];
//
//                    if (lengths.size() > 5) lengths.erase(lengths.begin());  // Keep last 5 segments
//                }
//
//                // Check for a valid 1:1:3:1:1 ratio
//                if (lengths.size() == 5 && is_approx_ratio(lengths, {1, 1, 3, 1, 1}, 0.3)) {
//                    int center_x = x - lengths[2] - lengths[3] - lengths[4] / 2; // Center of middle black block
//                    int center_y = y;
////                    patterns.emplace_back(Vector(center_x,center_y,0));
//                    // Verify vertically
//                    if (validate_vertical_ratio(binary_image, center_x, center_y)) {
//                        patterns.emplace_back(Vector(center_x, center_y, 0));
//                    }
//                }
//            }
//        }
cout << "Anzahl der Finder Patterns: " << patterns.size() << endl;
        return patterns;

}

// Function to check vertical 1:1:3:1:1 ratio
    bool QRdecoder::validate_vertical_ratio(const Img<bool> &binary_image, int x, int y) {
    vector<int> lengths;
    int height = binary_image.Height();

    // Start in the middle of the finder pattern
    int current = binary_image[y][x];


    // center segment
    // Expand upward
    int count_up = 0, count_down = 0;
    int i = y;
    while (i >= 0 && binary_image[i][x] == current) {
        count_up++;
        i--;
    }


    // Expand downward
    i = y + 1; // Start just below the center
    while (i < height && binary_image[i][x] == current) {
        count_down++;
        i++;
    }
    lengths.insert(lengths.begin(), count_up+count_down);

    current = !binary_image[y][x];


    //Middle 2 segments
    // Expand upward
    i =  y - count_up;
    count_up = 0;

    while (i >= 0 && binary_image[i][x] == current) {
        count_up++;
        i--;
    }
    if (count_up == 0) return false;
    lengths.insert(lengths.begin(), count_up);

    // Expand downward
    i = y + count_down;
    count_down = 0;
    while (i < height && binary_image[i][x] == current) {
        count_down++;
        i++;
    }
    if (count_down == 0) return false;
    lengths.push_back(count_down);

    //outer 2 segments
    i =  y - count_up;
    count_up = 0;

    while (i >= 0 && binary_image[i][x] == current) {
        count_up++;
        i--;
    }
    if (count_up == 0) return false;
    lengths.insert(lengths.begin(), count_up);

    // Expand downward
    i = y + count_down;
    count_down = 0;
    while (i < height && binary_image[i][x] == current) {
        count_down++;
        i++;
    }
    if (count_down == 0) return false;
    lengths.push_back(count_down);


//    // Now expand for remaining segments
//    for (int dir : {-1, 1}) { // -1 for up, 1 for down
//        int count = 0;
//        current = !binary_image[y][x]; // Flip color
//        i = (dir == -1) ? y - count_up : y + count_down;
//
//        while (i >= 0 && i < height) {
//            if (binary_image[i][x] == current) {
//                count++;
//            } else {
//                lengths.insert((dir == -1) ? lengths.begin() : lengths.end(), count);
//                count = 1;
//                current = binary_image[i][x];
//
//                if (lengths.size() == 5) break; // Stop once we have 5 segments
//            }
//            i += dir;
//        }
//    }

    // Ensure exactly 5 segments exist
    if (lengths.size() == 5) {
        return is_approx_ratio(lengths, {1,1,3,1,1}, 0.3);
    }
    return false;
    }

bool QRdecoder::resize(Img<RGB_Pixel> &img) {

    if ((imageHeight > 2000 || imageWidth > 2000)){
        img.Resize(imageWidth/2, imageHeight/2);
        imageWidth = img.Width();
        imageHeight = img.Height();
        return true;
    }
    return false;

}
