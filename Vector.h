#pragma once
#include <iostream>

class Vector
{
public:
    float X;
    float Y;
    float Z;

    Vector( float x, float y, float z);
    Vector();

    float dot(const Vector& v) const;
    float at(int i) const;
    int zeroIndex() const;
    Vector notVector() const;
    Vector cross(const Vector& v) const;
    Vector componentwiseMult(const Vector& v) const;
    Vector operator+(const Vector& v) const;
    Vector operator-(const Vector& v) const;
    Vector& operator+=(const Vector& v);
    Vector operator*(float c) const;
    Vector operator-() const;
    Vector& normalize();
    // Vector& max_element();
    // override == vector
    bool operator==(const Vector& v) const;
    bool operator!=(const Vector& v) const;
    Vector& toUnitVector();
    Vector& lerp(const Vector& other, float t);
    float length() const;
    float lengthSquared() const;
    void toString()const;
    Vector reflection( const Vector& normal) const;
    bool triangleIntersection( const Vector& d, const Vector& a, const Vector& b,
                               const Vector& c, float& s) const;
};
