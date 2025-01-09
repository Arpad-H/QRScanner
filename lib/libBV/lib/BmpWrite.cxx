//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück
//------------------------------------------------------------------------------------------
#include "BmpWrite.h"
#include "RGB_Pixel.h"

BmpWrite::BmpWrite(const char* FileName, const Img<RGB_Pixel>& img):
  FileOut(FileName), ilut(0)
{
   width  = 0;
   height = 0;
   *this = img;
}

BmpWrite::BmpWrite(const char* FileName, const Img<unsigned char>& img):
  FileOut(FileName), ilut(256)
{
   width  = 0;
   height = 0;
   for(int i=0;i<256;i++) {
     ilut[i]=RGB_Pixel(i,i,i);
   }
   *this = img;
}

BmpWrite::BmpWrite(const char* FileName, const Img<unsigned char>& img,vector<RGB_Pixel>& lut):
  FileOut(FileName), ilut(256)
{
   unsigned int i;
   width  = 0;
   height = 0;
   for (i=0;i<lut.size();i++) {
     ilut[i]=lut[i];
   }
   for(;i<256;i++) {
     ilut[i]=RGB_Pixel(0,0,0);
   }
   *this = img;
}

BmpWrite::BmpWrite(const char* FileName, const Img<bool>& img):
  FileOut(FileName), ilut(2)
{
   width  = 0;
   height = 0;
   ilut[0]=RGB_Pixel(0,0,0);
   ilut[1]=RGB_Pixel(255,255,255);
   *this = img;
}

BmpWrite::BmpWrite(const char* FileName, const Img<bool>& img, RGB_Pixel fg, RGB_Pixel bg):
  FileOut(FileName), ilut(2)
{
   width  = 0;
   height = 0;
   ilut[0]= bg;
   ilut[1]= fg;
   *this = img;
}

BmpWrite::~BmpWrite() {
}

int BmpWrite::operator=(const Img<RGB_Pixel>& img) {
   long BytesPerLine;
   long BufferSize;
   int x,y;
   unsigned char* buffer;
   width  = img.Width();
   height = img.Height();
   BytesPerLine = Align32(width*3);
   BufferSize = BytesPerLine*height;
   // Bitmap File Header
   write_buffer("BM", 2);
   write_long(14+40+BufferSize);
   write_short(0);
   write_short(0);
   write_long(14+40);
   // DIB Header
   write_long(40);
   write_long(width);
   write_long(height);
   write_short(1);
   write_short(24);
   write_long(0);
   write_long(BufferSize);
   write_long(0);
   write_long(0);
   write_long(0);
   write_long(0);
   // no LUT
   ;
   // write pixel data
   for (y=0;y<height;y++) {
      for (x=0;x<width;x++) {
         RGB_Pixel& p = const_cast<Img<RGB_Pixel>&>(img)[y][x];
         write_char(p.Blue());
         write_char(p.Green());
         write_char(p.Red());
         buffer +=3;
      }
      for (int i=BytesPerLine-3*(x);i>0;i--) {
         write_char(0);
      }
   }
   return 0;
}

int BmpWrite::operator=(const Img<unsigned char>& img) {
   long BytesPerLine;
   long BufferSize;
   long LutSize;
   int x,y;
   width  = img.Width();
   height = img.Height();
   BytesPerLine = Align32(width);
   BufferSize = BytesPerLine*height;
   LutSize = 4*256;
   // Bitmap File Header
   write_buffer("BM", 2);
   write_long(14+40+LutSize+BufferSize);
   write_short(0);
   write_short(0);
   write_long(14+40+LutSize);
   // DIB Header
   write_long(40);
   write_long(width);
   write_long(height);
   write_short(1);
   write_short(8);
   write_long(0);
   write_long(BufferSize);
   write_long(0);
   write_long(0);
   write_long(256);
   write_long(256);
   // write LUT
   for (int i=0;i<256;i++) {
     write_char(ilut[i].Blue());
     write_char(ilut[i].Green());
     write_char(ilut[i].Red());
     write_char(0);
   }
   // write pixel data
   for (y=0;y<height;y++) {
      for (x=0;x<width;x++) {
         unsigned char& p = const_cast<Img<unsigned char>&>(img)[y][x];
         write_char(p);
      }
      // Alignment
      for (int i=BytesPerLine-(x);i>0;i--) {
         write_char(0);
      }
   }
   return 0;
}

int BmpWrite::operator=(const Img<bool>& img) {
   long BytesPerLine;
   long BufferSize;
   long LutSize;
   int x,y;
   width  = img.Width();
   height = img.Height();
   BytesPerLine = (width%8==0) ? Align32(width/8) : Align32(width/8+1);
   BufferSize = BytesPerLine*height;
   LutSize = 4*2;
   // Bitmap File Header
   write_buffer("BM", 2);
   write_long(14+40+LutSize+BufferSize);
   write_short(0);
   write_short(0);
   write_long(14+40+LutSize);
   // DIB Header
   write_long(40);
   write_long(width);
   write_long(height);
   write_short(1);
   write_short(1);
   write_long(0);
   write_long(BufferSize);
   write_long(0);
   write_long(0);
   write_long(2);
   write_long(2);
   // write LUT
   for (int i=0;i<2;i++) {
     write_char(ilut[i].Blue());
     write_char(ilut[i].Green());
     write_char(ilut[i].Red());
     write_char(0);
   }
   // write pixel data
   for (y=0;y<height;y++) {
      for (x=0;x<width;) {
         unsigned char value=0;
         for (int i=0;i<8;i++) {
           bool& p = const_cast<Img<bool>&>(img)[y][x];
           value <<= 1;
           if ((x<width)&&(p)) { value += 1; }
           x++;
         }
         write_char(value);
      }
      // Alignment
      for (int i=BytesPerLine-(x)/8;i>0;i--) {
         write_char(0);
      }
   }
   return 0;
}

