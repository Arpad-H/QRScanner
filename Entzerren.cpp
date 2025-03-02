// ------------------------------------------------------------------------------------------------
// Programm "Entzerren"
//   Unterprogramme zum Entzerren von Bildpunkten und ganzen Bildern
//
// B. Lang, HS Osnabrueck
// Version Dezember 2016 (RH)
// ------------------------------------------------------------------------------------------------

#include <cmath>
#include <algorithm>
#include "RGB_Pixel.h"
#include "Img.h"
#include "Entzerren.h"

#define DELTA (0.00001)

// ------------------------------------------------------------------------------------------------
// Eine Rotationsmatrix aus einem Rodriguezvektor berechnen
// ------------------------------------------------------------------------------------------------
// Parameter:
// [in]  rotVect : Rodriguezvektor
// [out] R       : Rotationsmatrix
// ------------------------------------------------------------------------------------------------
void RotMat_from_Rodriguez(double R[3][3], const double rotVect[3])
{

    const double &rx(rotVect[0]);
    const double &ry(rotVect[1]);
    const double &rz(rotVect[2]);

    double Theta = sqrt(pow(rx, 2.0) + pow(ry, 2.0) + pow(rz, 2.0));

    double kx, ky, kz;
    if (0.0 == Theta) {
        kx = 1.0;
        ky = 0.0;
        kz = 0.0;
    } else {
        kx = rx / Theta;
        ky = ry / Theta;
        kz = rz / Theta;
    }

    R[0][0] = cos(Theta) + (1 - cos(Theta)) * pow(kx, 2.0);
    R[0][1] = -sin(Theta) * kz + (1 - cos(Theta)) * kx * ky;
    R[0][2] = sin(Theta) * ky + (1 - cos(Theta)) * kx * kz;
    R[1][0] = sin(Theta) * kz + (1 - cos(Theta)) * kx * ky;
    R[1][1] = cos(Theta) + (1 - cos(Theta)) * pow(ky, 2.0);
    R[1][2] = -sin(Theta) * kx + (1 - cos(Theta)) * ky * kz;
    R[2][0] = -sin(Theta) * ky + (1 - cos(Theta)) * kx * kz;
    R[2][1] = sin(Theta) * kx + (1 - cos(Theta)) * ky * kz;
    R[2][2] = cos(Theta) + (1 - cos(Theta)) * pow(kz, 2.0);

}

// ------------------------------------------------------------------------------------------------
// Einen Vektor von Punkten entzerren
// ------------------------------------------------------------------------------------------------
// Parameter:
// [out] Points      : Vektor mit Punkten im entzerrten Bild (first = x/u/Spalte, second = v y/v/Zeile)
// [in]  intrinsic   : Intrinsische Parameter der entzerrten Kamera
// [in]  Points_d    : Vektor mit Punkten im verzerrten Bild (first = x/u/Spalte, second = v y/v/Zeile)
// [in]  intrinsic_d : Intrinsische Parameter der verzerrten Kamera
// [in]  distCoeffs  : Verzerrungsparameter
// [in]  rotVect     : Rodriguezvektor
// ------------------------------------------------------------------------------------------------
void UndistorePoints(
        vector<pair<double, double> >& Points,
        const double intrinsic[3][3],
        const vector<pair<double, double> >& Points_d,
        const double intrinsic_d[3][3],
        const double distCoeffs[5],
        const double rotVect[3])
{
    Points.clear();
    const double &fx_d(intrinsic_d[0][0]);
    const double &fy_d(intrinsic_d[1][1]);
    const double &cx_d(intrinsic_d[0][2]);
    const double &cy_d(intrinsic_d[1][2]);
    const double &k1(distCoeffs[0]);
    const double &k2(distCoeffs[1]);
    const double &k3(distCoeffs[4]);
    const double &p1(distCoeffs[2]);
    const double &p2(distCoeffs[3]);
    const double &fx(intrinsic[0][0]);
    const double &fy(intrinsic[1][1]);
    const double &cx(intrinsic[0][2]);
    const double &cy(intrinsic[1][2]);

    double Ri[3][3];
    RotMat_from_Rodriguez(Ri, rotVect);
    // Matrix transponieren (invertieren der Rotationsmatrix)
    swap(Ri[0][1], Ri[1][0]);
    swap(Ri[0][2], Ri[2][0]);
    swap(Ri[1][2], Ri[2][1]);

    for (unsigned int p = 0; p < Points_d.size(); p++) {
        double uSS = Points_d[p].first;
        double vSS = Points_d[p].second;
        //img[int(vSS)][int(uSS)]=RGB_Pixel(0,0,255);
        double xSS = uSS / fx_d - cx_d / fx_d;
        double ySS = vSS / fy_d - cy_d / fy_d;
        double xS = xSS;
        double yS = ySS;
        for (int iter = 0; iter < 10; iter++) {
            double xS_2(xS * xS),   yS_2(yS * yS);
            double xS_3(xS * xS_2), yS_3(yS * yS_2);
            double xS_4(xS * xS_3), yS_4(yS * yS_3);
            double xS_5(xS * xS_4), yS_5(yS * yS_4);
            double xS_6(xS * xS_5), yS_6(yS * yS_5);

            double r2 = xS_2 + yS_2;
            double fktx = xS * ( 1 + k1 * r2 + k2 * pow(r2, 2.0) + k3 * pow(r2, 3.0) ) + p1 * 2.0 * xS * yS + p2 * ( r2 + 2.0 * xS_2 ) - xSS;
            double fkty = yS * ( 1 + k1 * r2 + k2 * pow(r2, 2.0) + k3 * pow(r2, 3.0) ) + p1 * ( r2 + 2.0 * yS_2 ) + p2 * 2.0 * xS * yS - ySS;
            double J[2][2];
            J[0][0] = 1.0 + k1 * ( 3.0 * xS_2 + yS_2 )
                      /* dfx/dx */  + k2 * ( 5.0 * xS_4 + 6.0 * xS_2 * yS_2 + yS_4 )
                      + k3 * ( 7.0 * xS_6 + 15.0 * xS_4 * yS_2 + 9.0 * xS_2 * yS_4 + yS_6 )
                      + p1 * 2.0 * yS
                      + p2 * 6.0 * xS;
            J[0][1] =       k1 * 2.0 * xS * yS
                            /* dfx/dy */  + k2 * ( 4.0 * xS_3 * yS + 4.0 * xS * yS_3 )
                            + k3 * ( 6.0 * xS_5 * yS + 12.0 * xS_3 * yS_3 + 6.0 * xS * yS_5 )
                            + p1 * 2.0 * xS
                            + p2 * 2.0 * yS;
            J[1][0] =       k1 * 2.0 * xS * yS
                            /* dfy/dx */  + k2 * ( 4.0 * xS_3 * yS + 4.0 * xS * yS_3 )
                            + k3 * ( 6.0 * xS_5 * yS + 12.0 * xS_3 * yS_3 + 6.0 * xS * yS_5 )
                            + p1 * 2.0 * xS
                            + p2 * 2.0 * yS;
            J[1][1] = 1.0 + k1 * ( xS_2 + 3.0 * yS_2 )
                      /* dfy/dy */  + k2 * ( xS_4 + 6.0 * xS_2 * yS_2 + 5.0 * yS_4 )
                      + k3 * ( xS_6 + 9.0 * xS_4 * yS_2 + 15.0 * xS_2 * yS_4 + 7.0 * yS_6 )
                      + p1 * 6.0 * yS
                      + p2 * 2.0 * xS;
            double Jinv_scale = (J[0][0] * J[1][1] - J[0][1] * J[1][0]);
            double Jinv[2][2] = { { J[1][1] / Jinv_scale, -J[0][1] / Jinv_scale},
                                  { -J[1][0] / Jinv_scale,  J[0][0] / Jinv_scale}
            };
            double dx = (Jinv[0][0] * fktx + Jinv[0][1] * fkty);
            double dy = (Jinv[1][0] * fktx + Jinv[1][1] * fkty);
            xS = xS - dx;
            yS = yS - dy;
            if ((fabs(dy) < DELTA) && (fabs(dx) < DELTA)) {
                //xSS=xS;
                break;
            }
        }
        double xnh = Ri[0][0] * xS + Ri[0][1] * yS + Ri[0][2] * 1.0; // Bildpunkt vom gedrehten Kamerakoordinatensystem
        double ynh = Ri[1][0] * xS + Ri[1][1] * yS + Ri[1][2] * 1.0; // in das senkrechte Kamerakoordinatensystem drehen.
        double znh = Ri[2][0] * xS + Ri[2][1] * yS + Ri[2][2] * 1.0;

        double xn = xnh / znh; // Normierte Bildebene der gedrehten Kamera auf die
        double yn = ynh / znh; // normierte Bildebene der senkrechten Kamera projizieren

        double u = fx * xn + cx; // Normierte Bildebene auf die normale Bildebene transformieren
        double v = fy * yn + cy;
        Points.push_back(pair<double, double>(u, v));
    }
}

// ------------------------------------------------------------------------------------------------
// Ein Bild entzerren
// ------------------------------------------------------------------------------------------------
// Parameter:
// [out] img         : Enzerrtes Bild
// [in]  intrinsic   : Intrinsische Parameter der entzerrten Kamera
// [in]  img_d       : Verzerrtes Bild
// [in]  intrinsic_d : Intrinsische Parameter der verzerrten Kamera
// [in]  distCoeffs  : Verzerrungsparameter
// [in]  rotVect     : Rodriguezvektor
// ------------------------------------------------------------------------------------------------
void UndistoreImage(
        Img<RGB_Pixel> &img,
        const double  intrinsic[3][3],
        Img<RGB_Pixel> &img_d,
        const double  intrinsic_d[3][3],
        const double distCoeffs[5],
        const double rotVect[3] )
{
    const double &fx_d(intrinsic_d[0][0]);
    const double &fy_d(intrinsic_d[1][1]);
    const double &cx_d(intrinsic_d[0][2]);
    const double &cy_d(intrinsic_d[1][2]);
    const double &k1(distCoeffs[0]);
    const double &k2(distCoeffs[1]);
    const double &k3(distCoeffs[4]);
    const double &p1(distCoeffs[2]);
    const double &p2(distCoeffs[3]);
    const double &fx(intrinsic[0][0]);
    const double &fy(intrinsic[1][1]);
    const double &cx(intrinsic[0][2]);
    const double &cy(intrinsic[1][2]);

    img.Margin_Constant(RGB_Pixel(0, 0, 0));

    double R[3][3];
    RotMat_from_Rodriguez(R, rotVect);

    for (unsigned int v = 0; v < img.Height(); v++) {
        for (unsigned int u = 0; u < img.Width(); u++) {
            // Index im verzerrten Bild berechnen
            double xn = (double(u) - cx) / fx - img.Width() / 2.0;
            double yn = (double(v) - cy) / fy - img.Height() / 2.0;
            double zn = 1.0;
            double xh = R[0][0] * xn + R[0][1] * yn + R[0][2] * zn; // Bildpunkt im gedrehten Kamerakoordinatensystem
            double yh = R[1][0] * xn + R[1][1] * yn + R[1][2] * zn;
            double zh = R[2][0] * xn + R[2][1] * yn + R[2][2] * zn;
            double x = xh / zh + img.Width() / 2.0;
            double y = yh / zh + img.Height() / 2.0;
            double r_2 = x * x + y * y; // Verzerrung
            double x_d = x * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + 2.0 * p1 * x * y + p2 * (r_2 + 2.0 * x * x);
            double y_d = y * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + p1 * (r_2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            double u_d = fx_d * x_d + cx_d; // Bildkoordinate im verzerrten Bild
            double v_d = fy_d * y_d + cy_d;
#if 1
            // Bilinear Interpolation
            double k00 = (int(v_d) + 1 - v_d) * (int(u_d) + 1 - u_d);
            double k01 = (int(v_d) + 1 - v_d) * (u_d - int(u_d));
            double k10 = (v_d - int(v_d))  * (int(u_d) + 1 - u_d);
            double k11 = (v_d - int(v_d))  * (u_d - int(u_d));

            int red   =   k00 * img_d[int(v_d)][int(u_d)].Red()
                          + k01 * img_d[int(v_d)][int(u_d) + 1].Red()
                          + k10 * img_d[int(v_d + 1)][int(u_d)].Red()
                          + k11 * img_d[int(v_d + 1)][int(u_d) + 1].Red() + 0.5;
            int green =   k00 * img_d[int(v_d)][int(u_d)].Green()
                          + k01 * img_d[int(v_d)][int(u_d) + 1].Green()
                          + k10 * img_d[int(v_d + 1)][int(u_d)].Green()
                          + k11 * img_d[int(v_d + 1)][int(u_d) + 1].Green() + 0.5;
            int blue  =   k00 * img_d[int(v_d)][int(u_d)].Blue()
                          + k01 * img_d[int(v_d)][int(u_d) + 1].Blue()
                          + k10 * img_d[int(v_d + 1)][int(u_d)].Blue()
                          + k11 * img_d[int(v_d + 1)][int(u_d) + 1].Blue() + 0.5;
            red   = (red < 0)   ? 0 : red;
            red   = (red > 255)   ? 255 : red;
            green = (green < 0) ? 0 : green;
            green = (green > 255) ? 255 : green;
            blue  = (blue < 0)  ? 0 : blue;
            blue  = (blue > 255)  ? 255 : blue;
            img[v][u] = RGB_Pixel(red, green, blue);
#else
            // Nearest neighbor interpolation
			img[v][u] = img_d[int(v_d + 0.5)][int(u_d + 0.5)];
#endif
        }
    }
}

void UndistoreImage(
        Img<bool> &img,
        const double  intrinsic[3][3],
        Img<bool> &img_d,
        const double  intrinsic_d[3][3],
        const double distCoeffs[5],
        const double rotVect[3] )
{
    const double &fx_d(intrinsic_d[0][0]);
    const double &fy_d(intrinsic_d[1][1]);
    const double &cx_d(intrinsic_d[0][2]);
    const double &cy_d(intrinsic_d[1][2]);
    const double &k1(distCoeffs[0]);
    const double &k2(distCoeffs[1]);
    const double &k3(distCoeffs[4]);
    const double &p1(distCoeffs[2]);
    const double &p2(distCoeffs[3]);
    const double &fx(intrinsic[0][0]);
    const double &fy(intrinsic[1][1]);
    const double &cx(intrinsic[0][2]);
    const double &cy(intrinsic[1][2]);

    img.Margin_Constant(false); // Set margins to false (black)

    double R[3][3];
    RotMat_from_Rodriguez(R, rotVect);

    for (unsigned int v = 0; v < img.Height(); v++) {
        for (unsigned int u = 0; u < img.Width(); u++) {
            // Index in distorted image
            double xn = (double(u) - cx) / fx - img.Width() / 2.0;
            double yn = (double(v) - cy) / fy - img.Height() / 2.0;
            double zn = 1.0;
            double xh = R[0][0] * xn + R[0][1] * yn + R[0][2] * zn;
            double yh = R[1][0] * xn + R[1][1] * yn + R[1][2] * zn;
            double zh = R[2][0] * xn + R[2][1] * yn + R[2][2] * zn;
            double x = xh / zh + img.Width() / 2.0;
            double y = yh / zh + img.Height() / 2.0;
            double r_2 = x * x + y * y; // Distortion factor
            double x_d = x * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + 2.0 * p1 * x * y + p2 * (r_2 + 2.0 * x * x);
            double y_d = y * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + p1 * (r_2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            double u_d = fx_d * x_d + cx_d; // Image coordinate in distorted image
            double v_d = fy_d * y_d + cy_d;

#if 1
            // Bilinear Interpolation for boolean images
            int u0 = int(u_d);
            int v0 = int(v_d);
            int u1 = u0 + 1;
            int v1 = v0 + 1;

            double w00 = (v1 - v_d) * (u1 - u_d);
            double w01 = (v1 - v_d) * (u_d - u0);
            double w10 = (v_d - v0) * (u1 - u_d);
            double w11 = (v_d - v0) * (u_d - u0);

            // Ensure indices are within bounds
            bool p00 = (u0 >= 0 && u0 < img_d.Width() && v0 >= 0 && v0 < img_d.Height()) ? img_d[v0][u0] : false;
            bool p01 = (u1 >= 0 && u1 < img_d.Width() && v0 >= 0 && v0 < img_d.Height()) ? img_d[v0][u1] : false;
            bool p10 = (u0 >= 0 && u0 < img_d.Width() && v1 >= 0 && v1 < img_d.Height()) ? img_d[v1][u0] : false;
            bool p11 = (u1 >= 0 && u1 < img_d.Width() && v1 >= 0 && v1 < img_d.Height()) ? img_d[v1][u1] : false;

            // Weighted sum to determine the new pixel value (threshold at 0.5)
            double value = w00 * p00 + w01 * p01 + w10 * p10 + w11 * p11;
            img[v][u] = (value >= 0.5);

#else
            // Nearest neighbor interpolation
            int u_nn = int(u_d + 0.5);
            int v_nn = int(v_d + 0.5);
            if (u_nn >= 0 && u_nn < img_d.Width() && v_nn >= 0 && v_nn < img_d.Height()) {
                img[v][u] = img_d[v_nn][u_nn];
            } else {
                img[v][u] = false;
            }
#endif
        }
    }
}
void UndistoreImage(
        Img<unsigned int> &img,
        const double  intrinsic[3][3],
        Img<unsigned int> &img_d,
        const double  intrinsic_d[3][3],
        const double distCoeffs[5],
        const double rotVect[3] )
{
    const double &fx_d = intrinsic_d[0][0];
    const double &fy_d = intrinsic_d[1][1];
    const double &cx_d = intrinsic_d[0][2];
    const double &cy_d = intrinsic_d[1][2];
    const double &k1 = distCoeffs[0];
    const double &k2 = distCoeffs[1];
    const double &k3 = distCoeffs[4];
    const double &p1 = distCoeffs[2];
    const double &p2 = distCoeffs[3];
    const double &fx = intrinsic[0][0];
    const double &fy = intrinsic[1][1];
    const double &cx = intrinsic[0][2];
    const double &cy = intrinsic[1][2];

    img.Margin_Constant(0); // Set margins to black (0)

    double R[3][3];
    RotMat_from_Rodriguez(R, rotVect);

    for (unsigned int v = 0; v < img.Height(); v++) {
        for (unsigned int u = 0; u < img.Width(); u++) {
            double xn = (double(u) - cx) / fx;
            double yn = (double(v) - cy) / fy;
            double zn = 1.0;
            double xh = R[0][0] * xn + R[0][1] * yn + R[0][2] * zn;
            double yh = R[1][0] * xn + R[1][1] * yn + R[1][2] * zn;
            double zh = R[2][0] * xn + R[2][1] * yn + R[2][2] * zn;
            double x = xh / zh;
            double y = yh / zh;
            double r_2 = x * x + y * y;
            double x_d = x * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + 2.0 * p1 * x * y + p2 * (r_2 + 2.0 * x * x);
            double y_d = y * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + p1 * (r_2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            double u_d = fx_d * x_d + cx_d;
            double v_d = fy_d * y_d + cy_d;

            // Bilinear Interpolation for unsigned int images
            int u0 = int(u_d);
            int v0 = int(v_d);
            int u1 = u0 + 1;
            int v1 = v0 + 1;

            double w00 = (v1 - v_d) * (u1 - u_d);
            double w01 = (v1 - v_d) * (u_d - u0);
            double w10 = (v_d - v0) * (u1 - u_d);
            double w11 = (v_d - v0) * (u_d - u0);

            // Ensure indices are within bounds
            unsigned int p00 = (u0 >= 0 && u0 < img_d.Width() && v0 >= 0 && v0 < img_d.Height()) ? img_d[v0][u0] : 0;
            unsigned int p01 = (u1 >= 0 && u1 < img_d.Width() && v0 >= 0 && v0 < img_d.Height()) ? img_d[v0][u1] : 0;
            unsigned int p10 = (u0 >= 0 && u0 < img_d.Width() && v1 >= 0 && v1 < img_d.Height()) ? img_d[v1][u0] : 0;
            unsigned int p11 = (u1 >= 0 && u1 < img_d.Width() && v1 >= 0 && v1 < img_d.Height()) ? img_d[v1][u1] : 0;

            // Compute the new pixel value using bilinear interpolation
            img[v][u] = static_cast<unsigned int>(w00 * p00 + w01 * p01 + w10 * p10 + w11 * p11);
        }
    }
}
