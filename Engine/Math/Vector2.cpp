#include "Vector2.h"
#include "LinearAlgebra.h"

const Vector2 Vector2::ORIGIN = Vector2(0, 0);

void Vector2::rot(double angle)
{
    *this = ::rotate(*this, angle);
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