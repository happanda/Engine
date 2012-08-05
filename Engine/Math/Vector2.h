#ifndef INCLUDE_VECTOR2
#define INCLUDE_VECTOR2

#include <math.h>

class Vector2
{
public:
   Vector2() : v1(0), v2(0) { }
   Vector2(double v1, double v2) : v1(v1), v2(v2) { }
   Vector2(const Vector2& vect) : v1(vect.v1), v2(vect.v2) { }
   double v1, v2;

   double norm2sq() const { return v1 * v1 + v2 * v2; }
   double norm2() const { return sqrt(norm2sq()); }
   void rot(double angle);
   void normalize2();
   void invX() { v1 = -v1; }
   void invY() { v2 = -v2; }
   Vector2 perpendicular() const { return Vector2(v2, -v1); }
   const Vector2& operator=(const Vector2& vect);
   static const Vector2 ORIGIN;
};

#endif