#include "Matrix3.h"

Matrix3::Matrix3()
{
   a[0][0] = 0; a[0][1] = 0; a[0][2] = 0;
   a[1][0] = 0; a[1][1] = 0; a[1][2] = 0;
   a[2][0] = 0; a[2][1] = 0; a[2][2] = 0;
}

Matrix3 Matrix3::Diag(double a11, double a22, double a33)
{
   Matrix3 m;
   m.a11() = a11;
   m.a22() = a22;
   m.a33() = a33;
   return m;
}

Matrix3 Matrix3::Eye()
{
   Matrix3 m;
   m.a11() = m.a22() = m.a33() = 1;
   return m;
}

const Matrix3& Matrix3::operator=(const Matrix3& m)
{
   a[0][0] = m.a[0][0]; a[0][1] = m.a[0][1]; a[0][2] = m.a[0][2];
   a[1][0] = m.a[1][0]; a[1][1] = m.a[1][1]; a[1][2] = m.a[1][2];
   a[2][0] = m.a[2][0]; a[2][1] = m.a[2][1]; a[2][2] = m.a[2][2];
   return *this;
}

double& Matrix3::a11() { return a[0][0]; }
double& Matrix3::a12() { return a[0][1]; }
double& Matrix3::a13() { return a[0][2]; }
double& Matrix3::a21() { return a[1][0]; }
double& Matrix3::a22() { return a[1][1]; }
double& Matrix3::a23() { return a[1][2]; }
double& Matrix3::a31() { return a[2][0]; }
double& Matrix3::a32() { return a[2][1]; }
double& Matrix3::a33() { return a[2][2]; }