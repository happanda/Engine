#ifndef INCLUDE_LINEAR_ALGEBRA
#define INCLUDE_LINEAR_ALGEBRA

#include <math.h>

class Vector2
{
public:
   Vector2() : v1(0), v2(0) { }
   Vector2(double v1, double v2) : v1(v1), v2(v2) { }
   Vector2(const Vector2& vect) : v1(vect.v1), v2(vect.v2) { }
   double v1, v2;

   inline double norm2sq() const { return v1 * v1 + v2 * v2; }
   inline double norm2() const { return sqrt(norm2sq()); }
   void rot(double angle);
   void normalize2();
   void invX() { v1 = -v1; }
   void invY() { v2 = -v2; }
   Vector2 perpendicular() { return Vector2(v2, -v1); }
   const Vector2& operator=(const Vector2& vect);
};

class Vector3
{
public:
   Vector3() : v1(0), v2(0), v3(0) { }
   Vector3(double v1, double v2, double v3) : v1(v1), v2(v2), v3(v3) { }
   Vector3(const Vector3& vect) : v1(vect.v1), v2(vect.v2), v3(vect.v3) { }
   double v1, v2, v3;

   inline double norm2sq() const { return v1 * v1 + v2 * v2 + v3 * v3; }
   inline double norm2() const { return sqrt(norm2sq()); }
   void invX() { v1 = -v1; }
   void invY() { v2 = -v2; }
   void invZ() { v3 = -v3; }
   void normalize2();
   const Vector3& operator=(const Vector3& vect);
   inline Vector3 cross(const Vector3& vect)
   {
      return Vector3(v2 * vect.v3 - v3 * vect.v2,
         v3 * vect.v1 - v1 * vect.v3,
         v1 * vect.v2 - v2 * vect.v1);
   }
};

class Matrix2
{
public:
   Matrix2() : a11(0), a12(0), a21(0), a22(0) { }
   Matrix2(double a11, double a12, double a21, double a22)
      : a11(a11), a12(a12), a21(a21), a22(a22) { }
   double a11, a12, a21, a22;

   const Matrix2& operator=(const Matrix2& m);
};

Vector2 rotate(Vector2 vect, double angle);

inline double operator*(const Vector2& vect1, const Vector2& vect2)
{ return vect1.v1 * vect2.v1 + vect1.v2 * vect2.v2; }

inline Vector2 operator*(const Vector2& vect, double d)
{ return Vector2(vect.v1 * d, vect.v2 * d); }

inline Vector2 operator+(const Vector2& vect1, const Vector2& vect2)
{ return Vector2(vect1.v1 + vect2.v1, vect1.v2 + vect2.v2); }

inline Vector2 operator-(const Vector2& vect1, const Vector2& vect2)
{ return Vector2(vect1.v1 - vect2.v1, vect1.v2 - vect2.v2); }

inline Vector2 operator-(const Vector2& vect)
{ return Vector2(-vect.v1, -vect.v2); }

inline Vector2 operator*(const Matrix2& m, const Vector2& v)
{ return Vector2(m.a11 * v.v1 + m.a12 * v.v2, m.a21 * v.v1 + m.a22 * v.v2); }

inline Vector2 operator*(const Vector2& v, const Matrix2& m)
{ return Vector2(m.a11 * v.v1 + m.a21 * v.v2, m.a12 * v.v1 + m.a22 * v.v2); }

inline Matrix2 operator+(const Matrix2& m1, const Matrix2& m2)
{ return Matrix2(m1.a11 + m2.a11, m1.a12 + m2.a12, m1.a21 + m2.a21, m1.a22 + m2.a22); }

inline Matrix2 operator-(const Matrix2& m1, const Matrix2& m2)
{ return Matrix2(m1.a11 - m2.a11, m1.a12 - m2.a12, m1.a21 - m2.a21, m1.a22 - m2.a22); }

class Segment
{
public:
   Vector2 head;
   Vector2 tail;
   Segment(): head(Vector2(0,0)), tail(Vector2(0,0)) { }
   Segment(Vector2 head, Vector2 tail): head(head), tail(tail) { }
   Segment(const Segment& segm): head(segm.head), tail(segm.tail) { }
   const Segment& operator=(const Segment& segm);
   inline double length()
   {
      Vector2 v = tail - head;
      return v.norm2();
   }
};

#endif