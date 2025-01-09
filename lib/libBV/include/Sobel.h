//------------------------------------------------------------------------------------------
// Funktionen zur Ermittlung von Kantenbildern
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück
//------------------------------------------------------------------------------------------
#include <utility>
#include <cmath>
using namespace std;

#include <Img.h>

//---------------------------------------------------------------------------
// Spezielle Ausgaberoutine für Polarbilder:
//---------------------------------------------------------------------------
template <typename Pixel>
Img<RGB_Pixel> &Polar_to_PseudoColor(Img<pair<Pixel,Pixel> >& GradientImage)  {
  static Img<RGB_Pixel> RGB_out;
  RGB_out.Resize(GradientImage.Width(),GradientImage.Height());

  Pixel& p0_mag = GradientImage[0][0].first;
  Pixel min = p0_mag;
  Pixel Max = p0_mag;
  for (unsigned int y=0;y<GradientImage.Height();y++) {
    for (unsigned int x=0;x<GradientImage.Width();x++) {
      Pixel& Pixel_mag = GradientImage[y][x].first;
      if      (Pixel_mag>Max) { Max = Pixel_mag; }
      else if (Pixel_mag<min) { min = Pixel_mag; }
    }
  }
  for (unsigned int x=0;x<GradientImage.Width();x++) {
    for (unsigned int y=0;y<GradientImage.Height();y++) {
      RGB_Pixel& rgb_Pixel = RGB_out[y][x];
      Pixel& Pixel_mag   = GradientImage[y][x].first;
      Pixel& Pixel_Phase = GradientImage[y][x].second;
      // Bei Betrag 0 ist die Grundfarbe weiss. Gradienten werden durch Farben steigender Intensität gekennzeichnet
      rgb_Pixel.Red(static_cast<unsigned char>(255.0*(2.0-(Pixel_mag-min)/(Max-min)*(-cos(Pixel_Phase)+1))*0.5)+0.5);
      rgb_Pixel.Green(static_cast<unsigned char>(255.0*(2.0-(Pixel_mag-min)/(Max-min)*(-cos(Pixel_Phase-2.0*M_PI/3.0)+1))*0.5)+0.5);
      rgb_Pixel.Blue(static_cast<unsigned char>(255.0*(2.0-(Pixel_mag-min)/(Max-min)*(-cos(Pixel_Phase-4.0*M_PI/3.0)+1))*0.5)+0.5);
    }
  }
  return RGB_out;
}

