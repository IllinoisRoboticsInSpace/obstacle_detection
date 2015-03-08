
/**THIS FILE PROVIDES USEFUL LINEAR ALGEBRA CONTAINERS AND FUNCTIONS FOR 3D MAPPING**/
//Written by Leon R. Frickensmith: leonfrickensmith@gmail.com

#ifndef LINEAR_HPP
#define LINEAR_HPP

#include <math.h> //for sqrt()

template<typename T>
struct Vec2
{
    Vec2() {}
    Vec2(T xi, T yi=0)
    {
        x = xi;
        y = yi;
    }
    float length() const
    {
        return sqrt(x*x+y*y);
    }
    void normalize()//sets Vec unit length to 1
    {
        float dist = length();
        x /= dist;
        y /= dist;
    }
    T x;
    T y;
};
template<typename T>
struct Vec3
{
    Vec3() {}
    Vec3(T xi, T yi=0, T zi=0)
    {
        x = xi;
        y = yi;
        z = zi;
    }
    Vec3<T> cross(const Vec3<T>& second) const//cross product with another Vec3 ORDER MATTERS
    {
        Vec3<T> cross;
        cross.x = y*second.z-z*second.y;
        cross.y = z*second.x-x*second.z;
        cross.z = x*second.y-y*second.x;
        return cross;
    }
    T dot(const Vec3<T>& other) const//dot product with another Vec3
    {
        return (x*other.x + y*other.y + z*other.z);
    }
    float length() const
    {
        return sqrt(x*x+y*y+z*z);
    }
    void normalize()//sets Vec unit length to 1
    {
        float dist = length();
        x /= dist;
        y /= dist;
        z /= dist;
    }
    T x;
    T y;
    T z;
};
/**Defines some useful Vector types**/
typedef Vec2<float> Vec2f;
typedef Vec2<int> Vec2i;
typedef Vec3<float> Vec3f;
typedef Vec3<int> Vec3i;


/** 3x3 Matrix**/
template<typename T>
struct Mat3
{
    Mat3()//sets up to be Identity Matrix by default
    {
        x1 = 0;
        x2 = 0;
        x3 = 0;
        y1 = 0;
        y2 = 0;
        y3 = 0;
        z1 = 0;
        z2 = 0;
        z3 = 0;
    }
    Mat3<T> operator+(const Mat3<T>& o) const//overloaded for addition by a Vec3
    {
        Mat3<T> result;
        result.x1 = x1+o.x1;
        result.x2 = x2+o.x2;
        result.x3 = x3+o.x3;

        result.y1 = y1+o.y1;
        result.y2 = y2+o.y2;
        result.y3 = y3+o.y3;

        result.z1 = z1+o.z1;
        result.z2 = z2+o.z2;
        result.z3 = z3+o.z3;
        return result;
    }
    Mat3<T> operator*(float c) const//overloaded for multiplication by a Vec3
    {
        Mat3<T> result;
        result.x1 = x1*c;
        result.x2 = x2*c;
        result.x3 = x3*c;

        result.y1 = y1*c;
        result.y2 = y2*c;
        result.y3 = y3*c;

        result.z1 = z1*c;
        result.z2 = z2*c;
        result.z3 = z3*c;
        return result;
    }
    Vec3<T> operator*(const Vec3<T>& coord) const//overloaded for multiplication by a Vec3
    {
        Vec3<T> result;//maybe this could be optimized by using the constructor of Vec3
        result.x = x1*coord.x + x2*coord.y + x3*coord.z;
        result.y = y1*coord.x + y2*coord.y + y3*coord.z;
        result.z = z1*coord.x + z2*coord.y + z3*coord.z;
        return result;
    }
    Mat3<T> operator*(const Mat3<T>& mat) const
    {
        Mat3<T> result;
        result.x1 = x1*mat.x1 + x2*mat.y1 + x3*mat.z1;
        result.y1 = y1*mat.x1 + y2*mat.y1 + y3*mat.z1;
        result.z1 = z1*mat.x1 + z2*mat.y1 + z3*mat.z1;

        result.x2 = x1*mat.x2 + x2*mat.y2 + x3*mat.z2;
        result.y2 = y1*mat.x2 + y2*mat.y2 + y3*mat.z2;
        result.z2 = z1*mat.x2 + z2*mat.y2 + z3*mat.z2;

        result.x3 = x1*mat.x3 + x2*mat.y3 + x3*mat.z3;
        result.y3 = y1*mat.x3 + y2*mat.y3 + y3*mat.z3;
        result.z3 = z1*mat.x3 + z2*mat.y3 + z3*mat.z3;
        return result;
    }
    void setSkewSymCrossProd(const Vec3<T>& vec)//sets this matrix to be the skew-symetric cross-product of vec
    {
        x1 = 0;
        x2 = -vec.z;
        x3 = vec.y;

        y1 = vec.z;
        y2 = 0;
        y3 = -vec.x;

        z1 = -vec.y;
        z2 = vec.x;
        z3 = 0;
    }
    T x1, x2, x3;
    T y1, y2, y3;
    T z1, z2, z3;
};
typedef Mat3<float> Mat3f;
typedef Mat3<int> Mat3i;


#endif // LINEAR_HPP
