#ifndef _POSITION_H
#define _POSITION_H
//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     FH Osnabrück    
//------------------------------------------------------------------------------------------

class Position {
 protected:
  int x,y;
 public:
  Position(): x(0), y(0) {}
  Position(const int x, const int y): x(x), y(y) {}
  Position(const Position &p):x(p.x),y(p.y) {}
  int get_x() const { return x; }
  int get_y() const { return y; }
  void set_x(int xn) { x=xn; }
  void set_y(int yn) { y=yn; }
  bool operator==(const Position& r) const    { return (x==r.x) && (y==r.y);  }
  bool operator!=(const Position& r) const    { return (x!=r.x) || (y!=r.y);  }
  Position operator+(const Position& r) const { return Position(x+r.x,y+r.y); }
  Position operator-(const Position& r) const { return Position(x-r.x,y-r.y); }
  Position& operator=(const Position& r)      { x=r.x; y=r.y; return *this; }
};

template <class ValueType>
class FilterCoefficient : public Position {
 protected:
  ValueType value;
 public:
  FilterCoefficient() : Position(), value(0) {}
  FilterCoefficient(const int x, const int y, const ValueType v) : Position(x,y), value(v) {}
  FilterCoefficient(const Position& p, const ValueType v) : Position(p), value(v) {}
  FilterCoefficient(const FilterCoefficient &f) : Position(f.x,f.y), value(f.value) {}
  ValueType get_value() const { return value; }
};

template <class ValueType>
class SinglePixel : public FilterCoefficient<ValueType> {
 public:
  SinglePixel() : FilterCoefficient<ValueType>() {}
  SinglePixel(const int x, const int y, const ValueType v) : FilterCoefficient<ValueType>(x,y,v) {}
  SinglePixel(const Position& p, const ValueType v) : FilterCoefficient<ValueType>(p,v) {}
  SinglePixel(const SinglePixel &f) : FilterCoefficient<ValueType>(f.x,f.y,f.value) {}
  const bool operator<(const SinglePixel& r) const { return FilterCoefficient<ValueType>::value<r.value; }
};
#endif
