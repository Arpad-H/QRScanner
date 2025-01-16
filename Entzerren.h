#ifndef ENTZERREN_H_INCLUDED
#define ENTZERREN_H_INCLUDED
// ------------------------------------------------------------------------------------------------
// Programm "Entzerren"
//   Prototypen der Unterprogramme zum Entzerren von Bildpunkten und
//   ganzen Bildern
//
// B. Lang, HS Osnabrueck
// Version Dezember 2016 (RH)
// ------------------------------------------------------------------------------------------------

#include <vector>
using namespace std;

#include "RGB_Pixel.h"
#include "Img.h"

// ------------------------------------------------------------------------------------------------
// Eine Rotationsmatrix aus einem Rodriguezvektor berechnen
// ------------------------------------------------------------------------------------------------
// Parameter:
// [in]  rotVect : Rodriguezvektor
// [out] R       : Rotationsmatrix
// ------------------------------------------------------------------------------------------------
void RotMat_from_Rodriguez(
        double R[3][3],
        const double rotVect[3]
);

// ------------------------------------------------------------------------------------------------
// Einen Bild entzerren
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
        const double intrinsic[3][3],
        Img<RGB_Pixel> &img_d,
        const double intrinsic_d[3][3],
        const double distCoeffs[5],
        const double rotVect[3]
);

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
        const double rotVect[3]
);

#endif // ENTZERREN_H_INCLUDED