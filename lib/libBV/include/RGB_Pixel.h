//------------------------------------------------------------------------------------------
// Class for Pixel Representation
// in RGB Color Space
//
// (c) Prof. Dr.-Ing. Bernhard Lang
//     FH Osnabrück    
//------------------------------------------------------------------------------------------
#ifndef _RGB_Pixel_h_
#define _RGB_Pixel_h_
class RGB_Pixel {
  unsigned char R,G,B;
 public:
  RGB_Pixel() : R(0),G(0),B(0) {}
  RGB_Pixel(unsigned char Red,unsigned char Green, unsigned char Blue) : R(Red),G(Green),B(Blue) {}
  RGB_Pixel(unsigned char value) : R(value),G(value),B(value) {}
  RGB_Pixel(bool value) : R(value?255:0),G(value?255:0),B(value?255:0) {}
  unsigned char Red  () const    { return R;   }
  unsigned char Green() const    { return G; }
  unsigned char Blue () const    { return B;  }
  void Red  (int value) { R = value; }
  void Green(int value) { G = value; }
  void Blue (int value) { B = value; }
  bool operator==(const RGB_Pixel& r) const { return (R==r.R) && (G==r.G) && (B==r.B); }
  RGB_Pixel& operator= (const RGB_Pixel &r) { R=r.R; G=r.G; B=r.B; return *this; }
  RGB_Pixel& init (RGB_Pixel &r) { R=r.R; G=r.G; B=r.B; return *this; }
  operator int ()            const { return (299*R+587*G+114*B)/1000; }
  operator unsigned int ()   const { return int(*this); }
  operator short ()          const { return int(*this); }
  operator unsigned short () const { return int(*this); }
  operator unsigned char ()  const { return int(*this); }
  operator double ()         const { return ((299.0*R+587.0*G+114.0*B)/1000.0)/255; }
  operator float ()          const { return double(*this); }
};

#endif
