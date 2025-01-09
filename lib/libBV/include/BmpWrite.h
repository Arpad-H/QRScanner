#ifndef _bmpwrite_h_
#define _bmpwrite_h_
//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück    
//------------------------------------------------------------------------------------------

#include <stdio.h>
#include "FileIO.h"
#include "Img.h"
#include "RGB_Pixel.h"
#include <vector>
using namespace std;

class BmpWrite: public FileOut, public Align  {
 private:
  // Komponenten
  int width;
  int height;
  vector<RGB_Pixel> ilut;
  // Private Methoden
  int operator=(const Img<RGB_Pixel>&);
  int operator=(const Img<unsigned char>&);
  int operator=(const Img<bool>&);
  BmpWrite(){}           // Defaultkonstruktor gesperrt
  BmpWrite(BmpWrite&):FileOut(){}  // Kopierkonstruktor gesperrt
public:
  BmpWrite(const char*, const Img<RGB_Pixel>&);
  BmpWrite(const char*, const Img<unsigned char>&);
  BmpWrite(const char*, const Img<unsigned char>&, vector<RGB_Pixel>&);
  BmpWrite(const char*, const Img<bool>&);
  BmpWrite(const char*, const Img<bool>&, RGB_Pixel fg, RGB_Pixel bg);
  ~BmpWrite();
};

#endif

