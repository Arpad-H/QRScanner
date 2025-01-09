//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück    
//------------------------------------------------------------------------------------------

#include "Grafik.h"

#if 0
#include <iostream>
ostream& operator<<(ostream& o, GrafikObjekt g) {
  o << "(Bezugspunkt:x=" << g.getX0()
                << ",y=" << g.getY0() << ")" << flush;
  return o;
}
#endif

void Punkt::draw(Img<RGB_Pixel>& I) {
  I[y0][x0] = c;
}

void Linie::draw (Img<RGB_Pixel>& I,
                  int haupt, int neben,
                  int h_delta, int n_delta,
                  int &h, int &n, int &x, int &y) {
  I[y][x] = c; // Anfangspunkt zeichenen
  int Zaehler = haupt/2; // Halben Schritt zur Rundung vorgeben
  for (int i=0;i<haupt;i++) { // Schritte auf der Hauptachse
    // Punkte für Nebenachse aufakkumulieren
    Zaehler += neben;
    if (Zaehler >= haupt) {
      // Wieder mal Schritt auf Nebenachse weitergehen
      Zaehler -= haupt;
      n += n_delta;
    }
    h += h_delta;    // Schritt auf Hauptachse gehen
    I[y][x] = c; // Punkt zeichenen
  }
}

void Linie::draw (Img<RGB_Pixel> &I) {
  int x(x0), y(y0); // aktueller Pixel x,y
  int xl(xe-x0);    // Länge x
  int yl(ye-y0);    // Länge y
  int stepx(1), stepy(1); // Schrittrichtung
  if (xl<0) { stepx = -1; xl=-xl; }
  if (yl<0) { stepy = -1; yl=-yl; }
  if (xl>=yl) { // X-Vorzugsrichtung
    draw(I, xl,yl,stepx,stepy,x,y,x,y); 
  } else {      // Y-Vorzugsrichtung
    draw(I, yl,xl,stepy,stepx,y,x,x,y);
  }
}

void Rechteck::draw (Img<RGB_Pixel> &I) {
  Linie(x0,y0,x0,ye,c).draw(I);
  Linie(x0,ye,xe,ye,c).draw(I);
  Linie(xe,ye,xe,y0,c).draw(I);
  Linie(xe,y0,x0,y0,c).draw(I);
}

void RechteckGefuellt::draw (Img<RGB_Pixel> &I) {
  Rechteck::draw(I);
  int xa,xb,ya,yb;
  if (x0<xe) {xa=x0;xb=xe;} else {xb=x0;xa=xe;}
  if (y0<ye) {ya=y0;yb=ye;} else {yb=y0;ya=ye;}
  for (int x=xa+1;x<xb;x++) {
    for (int y=ya+1;y<yb;y++) {
      I[y][x] = f;
    }
  }
}

#include "simplefont.h"
void Zeichen::draw (Img<RGB_Pixel> &I) {
  SimpleFont a(z);
  for (int y=0; y<SimpleFont::height(); y++) {
    for (int x=0; x<SimpleFont::width(); x++) {
      I[y0+y][x0+x] = ((a[y][x]) ? fg : bg);
    }
  }
}

void Textzeile::draw(Img<RGB_Pixel> &I) {
  int x=x0;
  for (int i=0;0!=s[i];i++) {
    Zeichen(x,y0,s[i],fg,bg).draw(I);
    x += SimpleFont::width();
  }
}
