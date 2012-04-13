#include "Matrix2.h"

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