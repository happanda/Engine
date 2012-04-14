#ifndef INCLUDE_MATRIX2
#define INCLUDE_MATRIX2

class Matrix2
{
public:
   Matrix2();
   Matrix2(double a11, double a12, double a21, double a22);
   double& a11() { return a[0][0]; }
   double& a12() { return a[0][1]; }
   double& a21() { return a[1][0]; }
   double& a22() { return a[1][1]; }

   const Matrix2& operator=(const Matrix2& m);

   double a[2][2];
};

#endif