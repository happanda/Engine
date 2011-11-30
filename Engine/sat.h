#ifndef INCLUDE_SAT
#define INCLUDE_SAT

#include "geometry.h"
#include "shape.h"

int check_collide(const rectangle& rectA, const rectangle& rectB,
                   Vector2& point, Segment& edge);

double find_min_intersection(const rectangle& rectA, const rectangle& rectB,
                   Vector2& point, Segment& edge);

int check_collide(const rectangle& rect, const circle& circ,
                   Vector2& point, Segment& edge);

#endif