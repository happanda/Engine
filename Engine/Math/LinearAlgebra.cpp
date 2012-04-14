#include "LinearAlgebra.h"

/* Arbitrary members */
Vector2 rotate(Vector2 vect, double angle)
{
   double acos = cos(angle);
   double asin = sin(angle);
   Matrix2 m(acos, -asin, asin, acos);
   return m * vect;
}

Matrix3 operator*(const Matrix3& m1, const Matrix3& m2)
{
   Matrix3 m3;
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
      {
         double sum = 0;
         for (int k = 0; k < 3; k++)
         {
            sum += m1.a[i][k] * m2.a[k][j];
         }
         m3.a[i][j] = sum;
      }
   return m3;
}

Matrix3 operator+(const Matrix3& m1, const Matrix3& m2)
{
   Matrix3 m3;
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         m3.a[i][j] = m1.a[i][j] + m2.a[i][j];
   return m3;
}

Matrix3 operator-(const Matrix3& m1, const Matrix3& m2)
{
   Matrix3 m3;
   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         m3.a[i][j] = m1.a[i][j] - m2.a[i][j];
   return m3;
}

Vector3 operator*(const Matrix3& m, const Vector3& v)
{
   return Vector3(m.a[0][0] * v.v1 + m.a[0][1] * v.v2 + m.a[0][2] * v.v1,
      m.a[1][0] * v.v1 + m.a[1][1] * v.v2 + m.a[1][2] * v.v1,
      m.a[2][0] * v.v1 + m.a[2][1] * v.v2 + m.a[2][2] * v.v1);
}

Vector3 operator*(const Vector3& v, const Matrix3& m)
{
   return Vector3(v.v1 * m.a[0][0] + v.v2 * m.a[1][0] + v.v3 * m.a[2][0],
      v.v1 * m.a[0][1] + v.v2 * m.a[1][1] + v.v3 * m.a[2][1],
      v.v1 * m.a[0][2] + v.v2 * m.a[1][2] + v.v3 * m.a[2][2]);
}