#ifndef _BmpRead_h_
#define _BmpRead_h_
//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück
//------------------------------------------------------------------------------------------

#define DEBUG_BMP_READ 0

//#include <stdio.h>
#include "FileIO.h"
#include "Img.h"
#include "RGB_Pixel.h"

class BmpRead: public FileIn, public Align {
private:
  int width;
  int height;
  Img<RGB_Pixel>* i;
  BmpRead& operator= (const BmpRead&);   // Zuweisung sperren
  BmpRead(const BmpRead&);               // Kopierkonstruktor sperren
public:
  BmpRead(const char* FileName);
  ~BmpRead() { delete i; }
  operator const Img<RGB_Pixel>& () const {  return (*i); }
  int Width()  const { return width;  }
  int Height() const { return height; }
  const BmpRead& operator>> (Img<RGB_Pixel>& img) const { img=*i; return *this; }
};
#endif

