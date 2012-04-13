#ifndef INCLUDE_MATRIX3
#define INCLUDE_MATRIX3

class Matrix3
{
public:
   Matrix3();
   double a[3][3];

   const Matrix3& operator=(const Matrix3& m);

   static Matrix3 Diag(double a11, double a22, double a33);
   static Matrix3 Eye();
   static Matrix3 Zero();

   double& a11();
   double& a12();
   double& a13();
   double& a21();
   double& a22();
   double& a23();
   double& a31();
   double& a32();
   double& a33();
};

#endif