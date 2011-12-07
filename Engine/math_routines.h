#ifndef INCLUDE_MATH_ROUTINES
#define INCLUDE_MATH_ROUTINES

#include <math.h>
#include "geometry.h"
#include "shape.h"

void get_points(rectangle rect, Vector2& p1, Vector2& p2,
                Vector2& p3, Vector2& p4, double indent);

inline bool same_direction(Vector2 v, Vector2 u)
{ return (v * u > 0); }

// computes a vector V, such that (V * codirect > 0)
Vector2 perpendicular(Vector2 vect, Vector2 codirect);
bool isOn(Vector2 point, Vector2 interval1, Vector2 interval2);
double project(Vector2 vect, Vector2 line);
Vector2 closest_point(Vector2 point, Segment segment);
double distance(Vector2 point, Segment segment);

#endif