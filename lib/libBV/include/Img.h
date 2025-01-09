//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück
//------------------------------------------------------------------------------------------

#ifndef _Img_h_
#define _Img_h_

#include "Position.h"

#if 1
class OutOfBounds {
 public:
  OutOfBounds(const char* filename, int linenumber):
    filename(filename), linenumber(linenumber) {}
  const char* filename;
  int   linenumber;
};
#endif

//------------------------------------------------------------------------------------------
// "class Img" (Template-Klasse)
//------------------------------------------------------------------------------------------
template <class Pixel> class Img {
 public:
  enum Randbehandlungen { Ausnahme_werfen, Konstanter_Wert,
                          Zyklisch_Fortsetzen, Gespiegelt_Fortsetzen, Randwert_Fortsetzen};
  class Line; // Eingebettete Klasse "Line"
  friend class Img::Line;
 protected:
  int high_y, high_x;
  Line** imagebuffer;              // Vektor von Zeigern auf "Line"-Objekte, in denen die Bildzeilen gespeichert sind
  Pixel margin_value;              // Wert für Randbehandlung mit konstantem Wert
  Line dummy;                      // Dummy-Zeile für Randbehandlung mit konstantem Wert
  Randbehandlungen Randbehandlung; // Art der Randbehandlung
 public:
  Img(const int width=0, const int height=0, Randbehandlungen Randbehandlung=Konstanter_Wert);
  Img(const Img& r);
  virtual ~Img();
  Img& operator= (const Img& r);
  Img& operator= (const Pixel& p);
  Line&  operator [] (const int i);
  const Line&  operator [] (const int i) const;
  Pixel& operator [] (const Position p) { return (*this)[p.get_y()][p.get_x()]; }
  void Resize(const int width, const int height);

  // Randbehandlung
  void Margin_Exception()              { Randbehandlung = Ausnahme_werfen; }
  void Margin_Constant(const Pixel &p) { Randbehandlung = Konstanter_Wert; margin_value = p; }
  void Margin_Cyclic()                 { Randbehandlung = Zyklisch_Fortsetzen; }
  void Margin_Mirror()                 { Randbehandlung = Gespiegelt_Fortsetzen; }
  void Margin_Continue()               { Randbehandlung = Randwert_Fortsetzen; }

  // Statusabfrage
  unsigned int     Width()           const { return high_x+1;  }
  unsigned int     Height()          const { return high_y+1; }
  Randbehandlungen Margin_Handling() const { return Randbehandlung; }
  const Pixel&     Margin_Value()    const { return margin_value; }

};

//------------------------------------------------------------------------------------------
// "class Img::Line" (Template-Klasse)
//   In die Img-Klasse eingebettet zur Verwaltung einer Zeile
//------------------------------------------------------------------------------------------
template<class Pixel>
class Img<Pixel>::Line {
  friend class Img<Pixel>; // Img-Objekten den Zugriff erlauben
 private:
  int           high;
  Pixel*        linebuffer;
  Img<Pixel>* pimage;
 public:
  Line(const int width, Img<Pixel>& image)
    : high(width-1), linebuffer(new Pixel[width]), pimage(&image)  { }
  Line(const Line &r, Img<Pixel>& image)
    : high(r.high), linebuffer(new Pixel[r.high+1]), pimage(&image) { *this = r; }
  ~Line() { delete [] linebuffer; }
  Line& operator= (const Line& r);
  Line& operator= (const Pixel& p);
  Pixel& operator [] (const int i);
  const Pixel& operator [] (const int i) const;
  void Resize(const int w);
};

//--------------------------------------------------------------------------------------------------
// Img<Pixel>::Line& Img<Pixel>::Line::operator= (const Line& r)
//   Zuweisungsoperator für eine Bildzeile
//--------------------------------------------------------------------------------------------------
// Zeilenlänge an Quelle anpassen
template<class Pixel> typename Img<Pixel>::Line& Img<Pixel>::Line::operator= (const Line& r) {
  if (high!=r.high) {
    // Bei unterschiedlich großen Bildern muss die Zielzeile zunächst an die Quellzeile angepasst werden.
    if (high!=r.high) {
      // Bei unterschiedlicher Zeilenlänge muss neuer Speicher vereinbart werden
      delete linebuffer;
      linebuffer = new Pixel[r.high+1];
    }
    // Untere und obere Grenze anpassen
    high = r.high;
  }
  // Bilder sind nun gleich groß, nun können die Zeilenpuffer direkt umkopiert werden
  Pixel *dst, *src;
  dst=linebuffer; src=r.linebuffer;
  for (int i=0; i<=high; i++) {
    *dst++ = *src++;
  }
  return *this;
}

//--------------------------------------------------------------------------------------------------
// Img<Pixel>::Line& Img<Pixel>::Line::operator= (const Pixel& p)
//   Zuweisungsoperator zur Initialisierung eine Bildzeile mit einem konstanten Wert
//--------------------------------------------------------------------------------------------------
template<class Pixel> typename Img<Pixel>::Line& Img<Pixel>::Line::operator= (const Pixel& p) {
  Pixel *dst=linebuffer;
  for (int i=0; i<=high; i++) {
    *dst++ = p;
  }
  return *this;
}

//--------------------------------------------------------------------------------------------------
// Pixel& Img<Pixel>::Line::operator []
//   Zugriff auf Pixel innerhalb einer Zeile
//     Hier wird die Randbehandlung rechts und links durchgeführt
//--------------------------------------------------------------------------------------------------
template <class Pixel>
Pixel& Img<Pixel>::Line::operator [] (const int i) {
  if ((i>=0)&&(i<=high)) {
    return linebuffer[i];
  } else {
    // Hier werden verschiedene Methoden der Randbehandlung realisiert
    int r, width;
    switch(pimage->Randbehandlung) {
      case Ausnahme_werfen:
        throw OutOfBounds(__FILE__,__LINE__);
      case Konstanter_Wert:
        return pimage->margin_value;
      case Zyklisch_Fortsetzen:
        width = high+1;
        r=(i)%width;
        if (r<0) { r += width; }
        return linebuffer[r];
      case Gespiegelt_Fortsetzen:
        width = high+1;
        r=(i)%width;
        if (r<0) { r += width; }
        return linebuffer[width-r-1];
      case Randwert_Fortsetzen:
        if (i<0) return linebuffer[0];
        else return linebuffer[high];
    }
  }
  throw OutOfBounds(__FILE__,__LINE__);
}
template <class Pixel>
const Pixel& Img<Pixel>::Line::operator [] (const int i) const {
  if ((i>=0)&&(i<=high)) {
    return linebuffer[i];
  } else {
    // Hier werden verschiedene Methoden der Randbehandlung realisiert
    int r, width;
    switch(pimage->Randbehandlung) {
      case Ausnahme_werfen:
        throw OutOfBounds(__FILE__,__LINE__);
      case Konstanter_Wert:
        return pimage->margin_value;
      case Zyklisch_Fortsetzen:
        width = high+1;
        r=(i)%width;
        if (r<0) { r += width; }
        return linebuffer[r];
      case Gespiegelt_Fortsetzen:
        width = high+1;
        r=(i)%width;
        if (r<0) { r += width; }
        return linebuffer[width-r-1];
      case Randwert_Fortsetzen:
        if (i<0) return linebuffer[0];
        else return linebuffer[high];
    }
  }
  throw OutOfBounds(__FILE__,__LINE__);
}

//--------------------------------------------------------------------------------------------------
// void Img<Pixel>::Line::Resize(const int w)
//   Größe einer Bildzeile verändern
//--------------------------------------------------------------------------------------------------
template <class Pixel>
void Img<Pixel>::Line::Resize(const int w) {
  if (high!=(w-1)) {
    // Bei unterschiedlich großen Bildern muss die Zielzeile zunächst an die Quellzeile angepasst werden.
    if ((high)!=(w-1)) {
      // Bei unterschiedlicher Zeilenlänge muss neuer Speicher vereinbart werden
      delete linebuffer;
      linebuffer = new Pixel[w];
    }
    // Grenze anpassen
    high = w-1;
  }
}

//--------------------------------------------------------------------------------------------------
// Img<Pixel>::Img(const int width, const int height, Randbehandlungen Randbehandlung)
//   Konstruktor für ein Img-Objekt
//     Anmerkung: "dummy"-Zeile muss Länge 0 haben, damit bei konstanter Randbehandlung immer
//     auf den Randwert "margin_value" zugegriffen wird.
//--------------------------------------------------------------------------------------------------
template <class Pixel> Img<Pixel>::Img(const int width, const int height,
                                           Randbehandlungen Randbehandlung):
    high_y(height-1), high_x(width-1), imagebuffer(new Line*[height]),
    dummy(0, *this), Randbehandlung(Randbehandlung)
{ for (int i=0;i<=high_y;i++) {
    imagebuffer[i] = new Line(width, *this);
  }
}

//--------------------------------------------------------------------------------------------------
// Img<Pixel>::Img(const Img &r)
//   Kopierkonstruktor der Img-Klasse
//     Anmerkung: "dummy"-Zeile muss Länge 0 haben, damit bei konstanter Randbehandlung immer
//     auf den Randwert "margin_value" zugegriffen wird.
//--------------------------------------------------------------------------------------------------
template <class Pixel> Img<Pixel>::Img(const Img &r):
    high_y(r.high_y),high_x(r.high_x),
    imagebuffer(new Line*[r.high_y+1]),
    margin_value(r.margin_value), dummy(0, *this),
    Randbehandlung(r.Randbehandlung)
{ for (int i=0;i<=high_y;i++) {
    imagebuffer[i] = new Line(const_cast<Img &>(r)[i], *this);
  }
}

//--------------------------------------------------------------------------------------------------
// Img<Pixel>::~Img()
//   Destruktor
//--------------------------------------------------------------------------------------------------
template <class Pixel> Img<Pixel>::~Img() {
  for (int i=0;i<=high_y;i++) {
    delete imagebuffer[i];
  }
  delete [] imagebuffer;
}

//--------------------------------------------------------------------------------------------------
// Img<Pixel>& Img<Pixel>::operator= (const Img& r)
//   Zuweisungsoperator der Img-Klasse
//--------------------------------------------------------------------------------------------------
template <class Pixel> Img<Pixel>& Img<Pixel>::operator= (const Img& r) {
  if (high_y!=r.high_y) {
    // Zeilenanzahl von Quell- und Zielbild sind unterschiedlich.
    // Somit muss das Zielbild zunächst an die Quelle angepasst und dabei neuer Speicher
    // vereinbart werden.
    for (int y=0; y<high_y+1;y++) {
      // Alte Bildzeilen löschen
      delete imagebuffer[y];
    }
    delete imagebuffer;                           // Alten Vektor auf Bildzeilen freigeben
    imagebuffer = new Line*[r.high_y+1];  // Neuen Vektor auf Bildzeilen holen
    for (int i=0;i<=r.high_y;i++) {
      // Zeilen holen und mit Quelle initialisieren
      imagebuffer[i] = new Line(const_cast<Img &>(r)[i], *this);
    }
    high_y=r.high_y;
    high_x=r.high_x;
  } else {
    // Bildern gleicher Zeilenanzahl bearbeiten
    if (high_y!=r.high_y) {
      // Grenzen anpassen
      high_y=r.high_y;
      high_x=r.high_x;
    }
    // Zeilen umkopieren
    Line **src = r.imagebuffer;
    Line **dst = imagebuffer;
    for (int i=0;i<=high_y;i++) {
      **dst++ = **src++; // Zuweisungsoperator der "Line"-Klasse verwenden
    }
  }
  return *this;
}

//--------------------------------------------------------------------------------------------------
// Img<Pixel>& Img<Pixel>::operator= (const Pixel& p)
//   Zuweisungsoperator der Img-Klasse für Initialisierung mit konstantem Wert
//--------------------------------------------------------------------------------------------------
template <class Pixel> Img<Pixel>& Img<Pixel>::operator= (const Pixel& p) {
  Line **dst = imagebuffer;
  for (int i=0;i<=high_y;i++) {
    **dst++ = p; // Zuweisungsoperator der "Line"-Klasse verwenden
  }
  return *this;
}

//--------------------------------------------------------------------------------------------------
// Img<Pixel>::Line& Img<Pixel>::operator [] (const int i)
//   Feldzugriffoperator, liefert ein "Line"-Objekt zurück.
//     Hier wird die Randbehandlung oben und unten durchgeführt.
//--------------------------------------------------------------------------------------------------
template <class Pixel> typename Img<Pixel>::Line& Img<Pixel>::operator [] (const int i) {
  if ((i>=0)&&(i<=high_y)) {
    return *imagebuffer[i];
  } else {
    // Hier werden verschiedene Methoden der Randbehandlung realisiert
    int r,height;
    switch(Randbehandlung) {
      case Ausnahme_werfen:
        // Ausnahme werfen
        throw OutOfBounds(__FILE__,__LINE__);
      case Konstanter_Wert:
        return dummy;
      case Zyklisch_Fortsetzen:
        height = high_y+1;
        r=(i)%height;
        if (r<0) { r += height; }
        return *imagebuffer[r];
      case Gespiegelt_Fortsetzen:
        height = high_y+1;
        r=(i)%height;
        if (r<0) { r += height; }
        return *imagebuffer[height-r-1];
      case Randwert_Fortsetzen:
        if (i<0) return *imagebuffer[0];
        else         return *imagebuffer[high_y];
    }
  }
  throw OutOfBounds(__FILE__,__LINE__);
}

template <class Pixel>
const typename Img<Pixel>::Line& Img<Pixel>::operator [] (const int i) const {
  if ((i>=0)&&(i<=high_y)) {
    return *imagebuffer[i];
  } else {
    // Hier werden verschiedene Methoden der Randbehandlung realisiert
    int r,height;
    switch(Randbehandlung) {
      case Ausnahme_werfen:
        // Ausnahme werfen
        throw OutOfBounds(__FILE__,__LINE__);
      case Konstanter_Wert:
        return dummy;
      case Zyklisch_Fortsetzen:
        height = high_y+1;
        r=(i)%height;
        if (r<0) { r += height; }
        return *imagebuffer[r];
      case Gespiegelt_Fortsetzen:
        height = high_y+1;
        r=(i)%height;
        if (r<0) { r += height; }
        return *imagebuffer[height-r-1];
      case Randwert_Fortsetzen:
        if (i<0) return *imagebuffer[0];
        else         return *imagebuffer[high_y];
    }
  }
  throw OutOfBounds(__FILE__,__LINE__);
}

//--------------------------------------------------------------------------------------------------
// void Img<Pixel>::Resize(const int width, const int height)
//   Größe eines Bildes verändern
//--------------------------------------------------------------------------------------------------
template <class Pixel>
void Img<Pixel>::Resize(const int width, const int height) {
  if (high_y!=(height-1)) {
    // Zeilenanzahl ändert sich.
    // Somit muss das Zielbild angepasst und dabei neuer Speicher vereinbart werden.
    for (int y=0; y<high_y+1;y++) {
      // Alte Bildzeilen löschen
      delete imagebuffer[y];
    }
    delete imagebuffer;                  // Alten Vektor auf Bildzeilen freigeben
    imagebuffer = new Line*[height];  // Neuen Vektor auf Bildzeilen holen
    for (int i=0;i<=height-1;i++) {
      // Zeilen mit der neuen Länge holen
      imagebuffer[i] = new Line(width, *this);
    }
    high_y = height-1;
    high_x = width-1;
  } else {
    //  Zeilenanzahl wird nicht verändert
    if (high_y!=height-1) {
      // Grenzen anpassen
      high_y = height-1;
      high_x = width-1;
    }
    // Zeilen anpassen
    for (int i=0;i<=high_y;i++) {
      imagebuffer[i]->Resize(width); // Länge des "Line"-Objekts anpassen
    }
  }
}


#endif
