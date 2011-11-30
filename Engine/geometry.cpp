
#include "geometry.h"
/* Vector2 members */
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

/* Vector3 members */
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

/* Matrix2 members */
const Matrix2& Matrix2::operator=(const Matrix2& m)
{
   a11 = m.a11;
   a12 = m.a12;
   a21 = m.a21;
   a22 = m.a22;
   return *this;
}

/* Arbitrary members */
Vector2 rotate(Vector2 vect, double angle)
{
   double acos = cos(angle);
   double asin = sin(angle);
   Matrix2 m(acos, -asin, asin, acos);
   return m * vect;
}

const Segment& Segment::operator=(const Segment& segm)
{
   head = segm.head;
   tail = segm.tail;
   return *this;
}