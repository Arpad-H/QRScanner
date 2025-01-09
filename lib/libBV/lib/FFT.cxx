//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     FH Osnabrück
//------------------------------------------------------------------------------------------
#include "FFT.h"

#ifndef M_PI
constexpr double M_PI = 4 * atan(-1);
#endif

void Bitreverse(vector<complex<double> > &complex_vector, int w);
void Bitreverse_2D(Img<complex<double> > &complex_image, int w, int h);

//-------------------------------------------------------------------------------------------------
// Bitreverse-Vertauschung der Elemente eines eindimensionalen Vektors
//-------------------------------------------------------------------------------------------------
void Bitreverse(vector<complex<double> > &complex_vector, int w) {
  //
  // Bitreverse Vertauschung der Vektorelemente
  //
  for (int x=0, xb=0; x<((1<<w)); x++) { // x ist linearer Zaehler
                                         // xb ist zugehöriger Bitreverse-Zaehler
    if (x<xb) {
      complex<double> &value1 = complex_vector[x];
      complex<double> &value2 = complex_vector[xb];
      complex<double> tmp;
      tmp = value1; value1 = value2; value2 = tmp; // Vektorelemente austauschen
    }
    // Bitreverse Zähler xb weiterschalten
    int k=(1<<w); /* Mit Bit w initialisieren */
    do {
      k >>= 1;  // nächstes Bit auswählen
      xb ^= k;  // Bit k invertieren
    } while ((0==(k&xb))&&(k>0));  // solange Übertrag vorhanden
  }
}

//-------------------------------------------------------------------------------------------------
// Bitreverse-Vertauschung der Elemente eines zweidimensionalen Bildes
//-------------------------------------------------------------------------------------------------
void Bitreverse_2D(Img<complex<double> > &complex_image, int w, int h) {
  //
  // Bitreverse Vertauschung in Zeilen- und Spaltenrichtung
  //
  for (int y=0, yb=0; y<((1<<h)); y++) { // y ist linearer Zeilenzaehler von 1 bis Bildhoehe-2
                                         // yb ist zugehöriger Bitreverse-Zeilenzaehler
    if (y<=yb) {
      for (int x=0, xb=0; x<((1<<w)); x++) { // x ist linearer Spaltenzaehler von 1 bis Bildbreite-2
                                             // xb ist zugehöriger Bitreverse-Spaltenzaehler
        if ((x<xb)&&(y<yb)) {
          complex<double> &pixel11 = complex_image[y][x];
          complex<double> &pixel12 = complex_image[y][xb];
          complex<double> &pixel21 = complex_image[yb][x];
          complex<double> &pixel22 = complex_image[yb][xb];
          complex<double> tmp;
          tmp = pixel11; pixel11 = pixel22; pixel22 = tmp; // Pixelwerte austauschen
          tmp = pixel12; pixel12 = pixel21; pixel21 = tmp; // Pixelwerte austauschen
        } else if ((x==xb)&&(y<yb)) {
          complex<double> &pixel1 = complex_image[y][x];
          complex<double> &pixel2 = complex_image[yb][x];
          complex<double> tmp;
          tmp = pixel1; pixel1 = pixel2; pixel2 = tmp; // Pixelwerte austauschen
        } else if ((x<xb)&&(y==yb)) {
          complex<double> &pixel1 = complex_image[y][x];
          complex<double> &pixel2 = complex_image[y][xb];
          complex<double> tmp;
          tmp = pixel1; pixel1 = pixel2; pixel2 = tmp; // Pixelwerte austauschen
        }
        // Bitreverse Zähler xb weiterschalten
        int k=(1<<w); /* Mit Bit w initialisieren */
        do {
          k >>= 1;  // nächstes Bit auswählen
          xb ^= k;  // Bit k invertieren
        } while ((0==(k&xb))&&(k>0));  // solange Übertrag vorhanden
      }
    }
    // Bitreverse Zähler yb weiterschalten
    int k=(1<<h); // Mit Bit h initialisieren
    do {
      k >>= 1;  // naechstes Bit auswählen
      yb ^= k;  // Bit k invertieren
    } while ((0==(k&yb))&&(k>0));  // solange Übertrag vorhanden
  }
}

static Img<complex<double> > return_image;

//-------------------------------------------------------------------------------------------------
// Zweidimensionale schnelle Fouriertransformation
//-------------------------------------------------------------------------------------------------
Img<complex<double> > &FFT_2D(Img<complex<double> > &original_image) {
  Img<complex<double> > &fft_image = return_image;
  // --------------------------------------------------------------------------------------------------
  // Bildhoehe und Bildbreite auf 2er-Potenz ueberpruefen
  // und Logarithmus von Breite und Hoehe ermitteln
  // --------------------------------------------------------------------------------------------------
  int w,h; // Logarithmus (Basis 2) der Breite und Hoehe
  { int W=original_image.Width()-1;
    int H=original_image.Height()-1;
    for ( w=0; W>0; W>>=1 ) { w++; }
    for ( h=0; H>0; H>>=1 ) { h++; }
    if (static_cast<unsigned int>(1<<w)!=original_image.Width())  { throw "FFT_2D: Breite ist *keine* Zweierpotenz ... :-("; }
    if (static_cast<unsigned int>(1<<h)!=original_image.Height()) { throw "FFT_2D: Hoehe ist *keine* Zweierpotenz ... :-(";  }
  }

  //
  // Originalbild übernehmen
  //
  fft_image = original_image;

  //
  // Bitreverse-Vertauschung in Zeilen- und Spaltenrichtung
  //
  Bitreverse_2D(fft_image,w,h);

  //
  // Zeilentransformationen
  //
  for (int l=0; l<w; l++) { // Schichten 0 bis w-1
    int L = 1<<l;
    complex<double> U(1.0,0.0);
    complex<double> W(cos(M_PI/L),-sin(M_PI/L));
    for (int j=0; j<L; j++) {
      for (int i=j; i<(1<<w); i+=L<<1){
        int ip = i+L;
        for (int y=0;y<(1<<h);y++) {
          complex<double> tmp  = fft_image[y][ip] * U;
          fft_image[y][ip] = fft_image[y][i] - tmp;
          fft_image[y][i]  = fft_image[y][i] + tmp;
        }
      }
      U = U*W;
    }
  }

  //
  // Spaltentransformationen
  //
  for (int l=0; l<h; l++) { // Schichten 0 bis h-1
    int L = 1<<l;
    complex<double> U(1.0,0.0);
    complex<double> W(cos(M_PI/L),-sin(M_PI/L));
    for (int j=0; j<L; j++) {
      for (int i=j; i<(1<<h); i+=L<<1){
        int ip = i+L;
        for (int x=0;x<(1<<w);x++) {
          complex<double> tmp  = fft_image[ip][x] * U;
          fft_image[ip][x] = fft_image[i][x] - tmp;
          fft_image[i][x]  = fft_image[i][x] + tmp;
        }
      }
      U = U*W;
    }
  }

  //
  // Werte normieren
  //
  complex<double> Height(1<<h);
  complex<double> Width(1<<w);
  for (int i=0; i<(1<<h); i++) { // Normierung
    for (int x=0;x<(1<<w);x++) {
      fft_image[i][x]  = fft_image[i][x]/(Height*Width);
    }
  }

  return fft_image;
}

//-------------------------------------------------------------------------------------------------
// Zweidimensionale schnelle inverse Fouriertransformation
//-------------------------------------------------------------------------------------------------
Img<complex<double> > &iFFT_2D(Img<complex<double> > &fft_image) {
  Img<complex<double> > &original_image = return_image;
  // --------------------------------------------------------------------------------------------------
  // Bildhoehe und Bildbreite auf 2er-Potenz ueberpruefen
  // und Logarithmus von Breite und Hoehe ermitteln
  // --------------------------------------------------------------------------------------------------
  int w,h; // Logarithmus (Basis 2) der Breite und Hoehe
  { int W=fft_image.Width()-1;
    int H=fft_image.Height()-1;
    for ( w=0; W>0; W>>=1 ) { w++; }
    for ( h=0; H>0; H>>=1 ) { h++; }
    if (static_cast<unsigned int>(1<<w)!=fft_image.Width())  { throw "iFFT_2D: Breite ist *keine* Zweierpotenz ... :-("; }
    if (static_cast<unsigned int>(1<<h)!=fft_image.Height()) { throw "iFFT_2D: Hoehe ist *keine* Zweierpotenz ... :-(";  }
  }

  //
  // FFT-Bild übernehmen
  //
  original_image = fft_image;

  //
  // Bitreverse-Vertauschung in Zeilen- und Spaltenrichtung
  //
  Bitreverse_2D(original_image,w,h);

  //
  // Zeilentransformationen
  //
  for (int l=0; l<w; l++) { // Schichten 0 bis w-1
    int L = 1<<l;
    complex<double> U(1.0,0.0);
    complex<double> W(cos(M_PI/L),sin(M_PI/L));
    for (int j=0; j<L; j++) {
      for (int i=j; i<(1<<w); i+=L<<1){
        int ip = i+L;
        for (int y=0;y<(1<<h);y++) {
          complex<double> tmp  = original_image[y][ip] * U;
          original_image[y][ip] = original_image[y][i] - tmp;
          original_image[y][i]  = original_image[y][i] + tmp;
        }
      }
      U = U*W;
    }
  }

  //
  // Spaltentransformationen
  //
  for (int l=0; l<h; l++) { // Schichten 0 bis h-1
    int L = 1<<l;
    complex<double> U(1.0,0.0);
    complex<double> W(cos(M_PI/L),sin(M_PI/L));
    for (int j=0; j<L; j++) {
      for (int i=j; i<(1<<h); i+=L<<1){
        int ip = i+L;
        for (int x=0;x<(1<<w);x++) {
          complex<double> tmp  = original_image[ip][x] * U;
          original_image[ip][x] = original_image[i][x] - tmp;
          original_image[i][x]  = original_image[i][x] + tmp;
        }
      }
      U = U*W;
    }
  }

  return original_image;
}

static vector<complex<double> > return_vector;

//-------------------------------------------------------------------------------------------------
// Eindimensionale schnelle Fouriertransformation
//-------------------------------------------------------------------------------------------------
vector<complex<double> > &FFT(vector<complex<double> > &original_vector) {
  vector<complex<double> > &fft_vector = return_vector;
  // --------------------------------------------------------------------------------------------------
  // Vektorgroesse auf 2er-Potenz ueberpruefen
  // und Logarithmus der Groesse ermitteln
  // --------------------------------------------------------------------------------------------------
  int w; // Logarithmus (Basis 2) der Größe
  { int W=original_vector.size()-1;
    for ( w=0; W>0; W>>=1 ) { w++; }
    if (static_cast<unsigned int>(1<<w)!=original_vector.size())  { throw "FFT: Größe ist *keine* Zweierpotenz ... :-("; }
  }

  fft_vector = original_vector;
  //
  // Bitreverse-Vertauschung der Vektorelemente
  //
  Bitreverse(fft_vector,w);

  //
  // Transformation
  //
  for (int l=0; l<w; l++) { // Schichten 0 bis w-1
    int L = 1<<l;
    complex<double> U(1.0,0.0);
    complex<double> W(cos(M_PI/L),-sin(M_PI/L));
    for (int j=0; j<L; j++) {
      for (int i=j; i<(1<<w); i+=L<<1){
        int ip = i+L;
        complex<double> tmp  = fft_vector[ip] * U;
        fft_vector[ip] = fft_vector[i] - tmp;
        fft_vector[i]  = fft_vector[i] + tmp;
      }
      U = U*W;
    }
  }

  //
  // Werte normieren
  //
  complex<double> Width(1<<w);
  for (int x=0;x<(1<<w);x++) {
    fft_vector[x]  = fft_vector[x]/Width;
  }

  return fft_vector;
}

//-------------------------------------------------------------------------------------------------
// Eindimensionale schnelle inverse Fouriertransformation
//-------------------------------------------------------------------------------------------------
vector<complex<double> > &iFFT(vector<complex<double> > &fft_vector) {
  vector<complex<double> > &original_vector = return_vector;
  // --------------------------------------------------------------------------------------------------
  // Vektorgroesse auf 2er-Potenz ueberpruefen
  // und Logarithmus der Groesse ermitteln
  // --------------------------------------------------------------------------------------------------
  int w; // Logarithmus (Basis 2) der Größe
  { int W=fft_vector.size()-1;
    for ( w=0; W>0; W>>=1 ) { w++; }
    if (static_cast<unsigned int>(1<<w)!=fft_vector.size())  { throw "FFT: Größe ist *keine* Zweierpotenz ... :-("; }
  }

  original_vector = fft_vector;
  //
  // Bitreverse-Vertauschung in Zeilen- und Spaltenrichtung
  //
  Bitreverse(original_vector,w);

  //
  // Rücktransformationen
  //
  for (int l=0; l<w; l++) { // Schichten 0 bis w-1
    int L = 1<<l;
    complex<double> U(1.0,0.0);
    complex<double> W(cos(M_PI/L),sin(M_PI/L));
    for (int j=0; j<L; j++) {
      for (int i=j; i<(1<<w); i+=L<<1){
        int ip = i+L;
        complex<double> tmp  = original_vector[ip] * U;
        original_vector[ip]   = original_vector[i] - tmp;
        original_vector[i]    = original_vector[i] + tmp;
      }
      U = U*W;
    }
  }

  return original_vector;
}

