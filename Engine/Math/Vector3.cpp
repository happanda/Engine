#include "Vector3.h"

void Vector3::normalize2()
{
   double norm = norm2();
   v1 /= norm;
   v2 /= norm;
   v3 /= norm;
}
const Vector3& Vector3::operator=(const Vector3& vect)
{
   v1 = vect.v1;
   v2 = vect.v2;
   v3 = vect.v3;
   return *this;
}