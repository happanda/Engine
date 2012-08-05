#ifndef INCLUDE_MATHROUTINES
#define INCLUDE_MATHROUTINES

#include <math.h>
#include "Math\Geometry.h"
#include "Body\Shape.h"

//void get_points(rectangle rect, Vector2& p1, Vector2& p2,
//                 Vector2& p3, Vector2& p4, double indent);

inline bool same_direction(Vector2 v, Vector2 u)
{ return (v * u > 0); }

// computes a vector V, such that (V * codirect > 0)
Vector2 perpendicular(const Vector2& vect, const Vector2& codirect);
bool isOn(const Vector2& point, const Vector2& interval1, const Vector2& interval2);
double project(const Vector2& vect, const Vector2& line);
Vector2 closest_point(const Vector2& point, const Segment& segment);
double distance(const Vector2& point, const Segment& segment);
// creates 3D vectors from vect1 and vect2 (z component is zero)
// and computes (v1 x v2) x v1, takes x and y components as resulting 2D vector
Vector2 cross_cross(const Vector2& vect1, const Vector2& vect2);

double clamp(double value, double minValue, double maxValue);

#endif