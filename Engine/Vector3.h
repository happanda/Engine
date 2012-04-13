#ifndef INCLUDE_VECTOR3
#define INCLUDE_VECTOR3

#include <math.h>

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

#endif