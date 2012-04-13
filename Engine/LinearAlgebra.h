#ifndef INCLUDE_LINEAR_ALGEBRA
#define INCLUDE_LINEAR_ALGEBRA

#include <math.h>
#include "Vector2.h"
#include "Vector3.h"
#include "Matrix2.h"
#include "Matrix3.h"

Matrix3 operator*(const Matrix3& m1, const Matrix3& m2);
Matrix3 operator+(const Matrix3& m1, const Matrix3& m2);
Matrix3 operator-(const Matrix3& m1, const Matrix3& m2);

Vector3 operator*(const Matrix3& m, const Vector3& v);
Vector3 operator*(const Vector3& v, const Matrix3& m);

Vector2 rotate(Vector2 vect, double angle);

inline double operator*(const Vector2& vect1, const Vector2& vect2)
{ return vect1.v1 * vect2.v1 + vect1.v2 * vect2.v2; }

inline Vector2 operator*(const Vector2& vect, double d)
{ return Vector2(vect.v1 * d, vect.v2 * d); }

inline Vector2 operator+(const Vector2& vect1, const Vector2& vect2)
{ return Vector2(vect1.v1 + vect2.v1, vect1.v2 + vect2.v2); }

inline Vector2 operator-(const Vector2& vect1, const Vector2& vect2)
{ return Vector2(vect1.v1 - vect2.v1, vect1.v2 - vect2.v2); }

inline Vector2 operator-(const Vector2& vect)
{ return Vector2(-vect.v1, -vect.v2); }

inline Vector2 operator*(const Matrix2& m, const Vector2& v)
{ return Vector2(m.a[0][0] * v.v1 + m.a[0][1] * v.v2, m.a[1][0] * v.v1 + m.a[1][1] * v.v2); }

inline Vector2 operator*(const Vector2& v, const Matrix2& m)
{ return Vector2(m.a[0][0] * v.v1 + m.a[1][0] * v.v2, m.a[0][1] * v.v1 + m.a[1][1] * v.v2); }

inline Matrix2 operator+(const Matrix2& m1, const Matrix2& m2)
{ return Matrix2(m1.a[0][0] + m2.a[0][0], m1.a[0][1] + m2.a[0][1], m1.a[1][0] + m2.a[1][0], m1.a[1][1] + m2.a[1][1]); }

inline Matrix2 operator-(const Matrix2& m1, const Matrix2& m2)
{ return Matrix2(m1.a[0][0] - m2.a[0][0], m1.a[0][1] - m2.a[0][1], m1.a[1][0] - m2.a[1][0], m1.a[1][1] - m2.a[1][1]); }


#endif