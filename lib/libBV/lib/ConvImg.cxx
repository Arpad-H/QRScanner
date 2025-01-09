//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück
//------------------------------------------------------------------------------------------

#include <utility>
#include <cmath>
using namespace std;

#include <Img.h>
#include <ConvImg.h>

#ifndef M_PI
constexpr double M_PI = 4 * atan(-1);
#endif

//---------------------------------------------------------------------------
// Spezielle Ausgaberoutine für Polarbilder:
//---------------------------------------------------------------------------
Img<RGB_Pixel> Polar_to_PseudoColor(Img<pair<double, double> >& GradientImage)  {
  // Ergebnisbild vereinbaren
  Img<RGB_Pixel> RGB_out(GradientImage.Width(),GradientImage.Height());
  // Minimalen und maximalen Betrag bestimmen
  double& p0_mag = GradientImage[0][0].first;
  double min = p0_mag;
  double Max = p0_mag;
  for (unsigned int y = 0; y < GradientImage.Height(); y++) {
    for (unsigned int x = 0; x < GradientImage.Width(); x++) {
      double& Pixel_mag = GradientImage[y][x].first;
      if (Pixel_mag > Max) {
        Max = Pixel_mag;
      } else if (Pixel_mag < min) {
        min = Pixel_mag;
      }
    }
  }
  // Gradientenbild in Farbbild konvertieren
  for (unsigned int x = 0; x < GradientImage.Width(); x++) {
    for (unsigned int y = 0; y < GradientImage.Height(); y++) {
      RGB_Pixel& rgb_Pixel = RGB_out[y][x];
      double& Pixel_mag   = GradientImage[y][x].first;
      double& Pixel_Phase = GradientImage[y][x].second;
      // Bei Betrag 0 ist die Grundfarbe weiss.
      // Der Winkel wird durch den Farbwert kodiert
      // Der Betrag wird durch steigende Saettigung kodiert
      rgb_Pixel.Red  (static_cast<unsigned char>(255.0 * (2.0 - (Pixel_mag - min) / (Max - min)*(-cos(Pixel_Phase) + 1)) * 0.5));
      rgb_Pixel.Green(static_cast<unsigned char>(255.0 * (2.0 - (Pixel_mag - min) / (Max - min)*(-cos(Pixel_Phase - 2.0 * M_PI / 3.0) + 1)) * 0.5));
      rgb_Pixel.Blue (static_cast<unsigned char>(255.0 * (2.0 - (Pixel_mag - min) / (Max - min)*(-cos(Pixel_Phase - 4.0 * M_PI / 3.0) + 1)) * 0.5));
    }
  }
  return RGB_out;
}
