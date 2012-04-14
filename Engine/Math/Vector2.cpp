#include "Vector2.h"

const Vector2 Vector2::ORIGIN = Vector2(0, 0);

void Vector2::rot(double angle)
{
   double acos = cos(angle);
   double asin = sin(angle);
   v1 = v1 * acos - v2 * asin;
   v2 = v1 * asin + v2 * acos;
}
void Vector2::normalize2()
{
   double norm = norm2();
   v1 /= norm;
   v2 /= norm;
}
const Vector2& Vector2::operator=(const Vector2& vect)
{
   v1 = vect.v1;
   v2 = vect.v2;
   return *this;
}