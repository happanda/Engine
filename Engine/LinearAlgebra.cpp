#include "LinearAlgebra.h"

/* Vector2 members */
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

/* Arbitrary members */
Vector2 rotate(Vector2 vect, double angle)
{
   double acos = cos(angle);
   double asin = sin(angle);
   Matrix2 m(acos, -asin, asin, acos);
   return m * vect;
}

/* Matrix2 members */
Matrix2::Matrix2()
{
   a[0][0] = 0;
   a[0][1] = 0;
   a[1][0] = 0;
   a[1][1] = 0;
}
Matrix2::Matrix2(double a11, double a12, double a21, double a22)
{
   a[0][0] = a11;
   a[0][1] = a12;
   a[1][0] = a21;
   a[1][1] = a22;
}

const Matrix2& Matrix2::operator=(const Matrix2& m)
{
   a[0][0] = m.a[0][0];
   a[0][1] = m.a[0][1];
   a[1][0] = m.a[1][0];
   a[1][1] = m.a[1][1];
   return *this;
}
