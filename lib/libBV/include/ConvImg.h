//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück
//------------------------------------------------------------------------------------------

#ifndef _ConvImg_h_
#define _ConvImg_h_

#include <utility>
using namespace std;

#include <Img.h>
#include <RGB_Pixel.h>

template<typename DPixel, typename SPixel>
Img<DPixel> ConvImg(const Img<SPixel> &src) {
  // Ergebnisbild vereinbaren
  Img<DPixel> dst(src.Width(),src.Height());
  // Konvertierung durchfuehren
  for (unsigned int y=0; y<src.Height(); y++ ) {
    for (unsigned int x=0; x<src.Width(); x++ ) {
      const SPixel& p = src[y][x];
      dst[y][x] = static_cast<DPixel>(p); // Konversion von SPixel nach DPixel, Kopierzuweisung
    }
  }
  return dst;
}

template<typename DPixel, typename SPixel>
Img<DPixel> ConvImg(const Img<SPixel> &src, const SPixel scale, const SPixel offset) {
  // Ergebnisbild vereinbaren
  Img<DPixel> dst(src.Width(),src.Height());
  // Konvertierung durchfuehren
  for (unsigned int y=0; y<src.Height(); y++ ) {
    for (unsigned int x=0; x<src.Width(); x++ ) {
      const SPixel& p = src[y][x];
      dst[y][x] = static_cast<DPixel>((p+offset)*scale); // SPixel: Multiplikation und Addition
                                                         // Konversion von SPixel nach DPixel, Kopierzuweisung
    }
  }
  return dst;
}

template<typename fPixel, typename sPixel>
Img<fPixel> get_first(Img<pair<fPixel,sPixel> >& src) {
  // Ergebnisbild vereinbaren
  Img<fPixel> dst(src.Width(),src.Height());
  // first-Bild extrahieren
  for (unsigned int y=0; y<src.Height(); y++ ) {
    for (unsigned int x=0; x<src.Width(); x++ ) {
      const pair<fPixel,sPixel>& p = src[y][x];
      dst[y][x] = p.first;
    }
  }
  return dst;
}

template<typename fPixel, typename sPixel>
Img<fPixel> get_second(Img<pair<fPixel,sPixel> >& src) {
  // Ergebnisbild vereinbaren
  Img<sPixel> dst(src.Width(),src.Height());
  // second-Bild extrahieren
  for (unsigned int y=0; y<src.Height(); y++ ) {
    for (unsigned int x=0; x<src.Width(); x++ ) {
      const pair<fPixel,sPixel>& p = src[y][x];
      dst[y][x] = p.second;
    }
  }
  return dst;
}

template<typename Pixel>
pair<Pixel,Pixel> get_min_Max(const Img<Pixel> &img) {
  // Ergebnisbild vereinbaren
  Pixel min(img[0][0]), Max(img[0][0]);            // Pixel: Kopierkonstruktor
  // Minimal- und Maximalwert bestimmen
  for (unsigned int y=0; y<img.Height(); y++ ) {
    for (unsigned int x=0; x<img.Width(); x++ ) {
      const Pixel& p = img[y][x];
      if      (p<min) { min=p; }                   // Pixel: operator <
      else if (Max<p) { Max=p; }                   //
    }
  }
  return pair<Pixel,Pixel>(min,Max);
}

// Spezielle Ausgaberoutine fuer Gradientenbilder in Polardarstellung
Img<RGB_Pixel> Polar_to_PseudoColor(Img<pair<double, double> >& GradientImage);

#endif
