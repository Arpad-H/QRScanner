#include "vector.h"
#include <assert.h>
#include <math.h>

#define EPSILON 1e-6
//using namespace Vector;

Vector::Vector(float x, float y, float z) {
    X = x;
    Y = y;
    Z = z;
}

Vector::Vector() {

}

float Vector::dot(const Vector& v) const
{
    return v.X * X + v.Y * Y + v.Z * Z;
}

float Vector::at(int i) const
{
    switch (i)
    {
        case 0:
            return X;
            break;
        case 1:
            return Y;
            break;
        case 2:
            return Z;
            break;
        default:
            break;
    }
}
//helper function to get the index of the non zero value of a vector
//due to the nature if rightangled movement, in some cases we can assume that
//the vector will only have one non zero value
int Vector::zeroIndex() const
{
    if (X == 0)
    {
        return 0;
    }
    else if (Y == 0)
    {
        return 1;
    }
    else if (Z == 0)
    {
        return 2;
    }
    else
    {
        return -1;
    }
}


Vector Vector::notVector() const
{
    if(X != 0)
    {
        return Vector(0,1,1);
    }
    else if(Y != 0)
    {
        return Vector(1,0,1);
    }
    else if(Z != 0)
    {
        return Vector(1,1,0);
    }
    else
    {
        return Vector(0,0,0);
    }
    return Vector();
}

Vector Vector::cross(const Vector& v) const {
    Vector result = Vector((Y * v.Z) - (Z * v.Y),
                           (Z * v.X) - (X * v.Z),
                           (X * v.Y) - (Y * v.X));
    return result;
}

Vector Vector::componentwiseMult(const Vector& v) const
{
    return Vector(X * v.X, Y * v.Y, Z * v.Z);
}


Vector Vector::operator+(const Vector& v) const {

    return Vector(X + v.X,
                  Y + v.Y,
                  Z + v.Z);
}

Vector Vector::operator-(const Vector& v) const {

    return Vector(X - v.X,
                  Y - v.Y,
                  Z - v.Z);
}

Vector Vector::operator*(float c) const {
    return Vector(X * c,
                  Y * c,
                  Z * c);
}

Vector Vector::operator-() const {

    return Vector(-X,
                  -Y,
                  -Z);
}


Vector& Vector::operator+=(const Vector& v) {
    this->X += v.X;
    this->Y += v.Y;
    this->Z += v.Z;
    return *this;
}


Vector& Vector::normalize() {
    float length = this->length();
    this->X /= length;
    this->Y /= length;
    this->Z /= length;
    return *this;
}



bool Vector::operator==(const Vector& v) const
{
    return (X == v.X && Y == v.Y && Z == v.Z);
}

bool Vector::operator!=(const Vector& v) const
{
    return !(this->operator==(v));
}

Vector& Vector::toUnitVector()
{
    //convert a vector to a unit vector based on biggest value
    // it works in this application because we are only using it 
    // on direction vectors (up,forward,right) which are always
    // going to be 1 in one direction and 0 in the others (due to 4 directional movement)
    // avoids floating point precision errors
    float max = abs(X);
    if (abs(Y) > max) {
        max = abs(Y);
    }
    if (abs(Z) > max) {
        max = abs(Z);
    }
    if (abs(X) == max ) {
        this->X = copysign(1.0,X);
        this->Y = 0;
        this->Z = 0;
    }
    else if (abs(Y) == max) {
        this->X = 0;
        this->Y = copysign(1.0, Y);
        this->Z = 0;
    }
    else if (abs(Z) == max) {
        this->X = 0;
        this->Y = 0;
        this->Z = copysign(1.0, Z);
    }
    return *this;
}

//Vector& Vector::lerp(const Vector& other, float t)
//{
//    return Vector(
//            X * (1.0f - t) + other.X * t,
//            Y * (1.0f - t) + other.Y * t,
//            Z * (1.0f - t) + other.Z * t
//    );
//}

float Vector::length() const {

    return sqrt(lengthSquared());
}

float Vector::lengthSquared() const {

    return X * X + Y * Y + Z * Z;
}

Vector Vector::reflection(const Vector& normal) const {
    float dotProduct = dot(normal);
    Vector v = Vector(2 * dotProduct * normal.X,
                      2 * dotProduct * normal.Y,
                      2 * dotProduct * normal.Z);
    return *this - v;
}

void Vector::toString() const{
    std::cout << "x: " <<this->X << " y: " << this->Y << " z: " << this->Z << std::endl;
}

bool Vector::triangleIntersection(const Vector& d, const Vector& a, const Vector& b, const Vector& c, float& s) const {


    Vector ab = b - a;
    Vector ac = c - a;
    Vector normal = (ab.cross(ac)).normalize();

    //s = skalar von geraden um den schnittpunkt zu finden
    s = (a - *this).dot(normal) / d.dot(normal);
    if (s <= 0) {
        return false;
    }
    Vector intersectionPoint = *this + d * s;

    //Calculate Triangle Areas
    float area_abc = ab.cross(ac).length() / 2;
    float area_abp = ab.cross(intersectionPoint - a).length() / 2;
    float area_acp = ac.cross(intersectionPoint - a).length() / 2;
    float area_bcp = (b - c).cross(intersectionPoint - b).length() / 2;
    if (area_abc - (area_acp + area_abp + area_bcp) >= 0 - EPSILON) {
        return true;
    }
    return false;


}