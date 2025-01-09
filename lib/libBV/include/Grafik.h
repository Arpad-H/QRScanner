//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück    
//------------------------------------------------------------------------------------------
#ifndef _grafik_h_
#define _grafik_h_

#include "RGB_Pixel.h"
#include "Img.h"

class GrafikElement {
 protected:
  int x0, y0;       // Bezugspunkt
  GrafikElement(int x,int y):x0(x),y0(y) {}
 public:
  int getX0() const { return x0; }
  int getY0() const { return y0; }
  virtual void add_offset(int x, int y) { x0+=x; y0+=y; }
  virtual void draw(Img<RGB_Pixel>& I) {};
};

class Punkt: public GrafikElement {
  RGB_Pixel c;      // Farbe
 public:
  Punkt(int x0, int y0, RGB_Pixel color) :
    GrafikElement(x0,y0), c(color) {}
  const RGB_Pixel& getColor() const { return c; }
  virtual void draw(Img<RGB_Pixel>& I);
};

class Linie: public GrafikElement {
  RGB_Pixel c;      // Farbe
  int xe, ye;       // Linienendpunkt
  void draw (Img<RGB_Pixel>&, int,int, int,int, int&,int&, int&, int&);
 public:
  Linie(int x0, int y0, int x, int y, RGB_Pixel color) :
    GrafikElement(x0,y0), c(color), xe(x), ye(y) {}
  const RGB_Pixel& getColor() const { return c; }
  int getXe() const { return xe; }
  int getYe() const { return ye; }
  virtual void add_offset(int x, int y) { xe+=x; ye+=y; GrafikElement::add_offset(x,y); }
  virtual void draw(Img<RGB_Pixel>& I);
};

class Rechteck: public GrafikElement {
 protected:
  RGB_Pixel c;      // Farbe
  int xe, ye;       // Gegenüberliegender Punkt
 public:
  Rechteck(int x0, int y0, int x, int y, RGB_Pixel color) :
    GrafikElement(x0,y0), c(color), xe(x), ye(y) {}
  const RGB_Pixel& getColor() const { return c; }
  int getXe() const { return xe; }
  int getYe() const { return ye; }
  virtual void add_offset(int x, int y) { xe+=x; ye+=y; GrafikElement::add_offset(x,y); }
  virtual void draw(Img<RGB_Pixel>& I);
};

class Zeichen: public GrafikElement {
  RGB_Pixel fg;     // Vordergrundfarbe
  RGB_Pixel bg;     // Hintergrundfarbe
  unsigned char z;  // Zeichencode
 public:
  Zeichen(int x0, int y0, unsigned char z, RGB_Pixel fg, RGB_Pixel bg) :
    GrafikElement(x0,y0), fg(fg), bg(bg), z(z) {}
  const RGB_Pixel& getFGColor() const { return fg; }
  const RGB_Pixel& getBGColor() const { return bg; }
  unsigned char getZ() const { return z; }
  virtual void draw(Img<RGB_Pixel>& I);
};

class Textzeile: public GrafikElement {
  RGB_Pixel fg;      // Vordergrundfarbe
  RGB_Pixel bg;      // Hintergrundfarbe
  const char *s;  // Zeichenstring
 public:
  Textzeile(int x0, int y0, const char* s, RGB_Pixel fg, RGB_Pixel bg) :
    GrafikElement(x0,y0), fg(fg), bg(bg), s(s) {}
  const RGB_Pixel& getFGColor() const { return fg; }
  const RGB_Pixel& getBGColor() const { return bg; }
  const char *getS() const { return s; }
  virtual void draw(Img<RGB_Pixel>& I);
};


class RechteckGefuellt: public Rechteck {
  RGB_Pixel f;      // Füllfarbe
 public:
  RechteckGefuellt(int x0, int y0, int x, int y,
                   RGB_Pixel lc, RGB_Pixel fc) :
    Rechteck(x0,y0,x,y,lc), f(fc){}
  const RGB_Pixel& getFillColor() const { return f; }
  virtual void draw(Img<RGB_Pixel>& I);
};

#endif
