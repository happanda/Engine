#ifndef INCLUDE_SHAPE
#define INCLUDE_SHAPE

#include "geometry.h"

enum shape_type
{
   sh_rectangle,
   sh_circle,
   sh_surface
};

struct shape
{
public:
   Vector2 point;// center of mass
   double alpha;// rotation angle
   shape_type type;
   void rotate(double angle)
   { alpha += angle; }
   shape(): point(Vector2(0, 0)), alpha(0) { }
   shape(shape_type sh, double point_x, double point_y,
      double alpha): type(sh), point(Vector2(point_x, point_y)), alpha(alpha) { }
};

struct rectangle : public shape
{
public:
   double h, w;// sides lengths
   rectangle(): shape(), h(1), w(1) { }
   rectangle(double point_x, double point_y, double alpha,
      double height, double width): shape(sh_rectangle, point_x, point_y, alpha),
      h(height), w(width) { }
};

struct circle : public shape
{
public:
   circle() { type = sh_circle; }
   circle(double point_x, double point_y, double alpha, double radius):
      shape(sh_circle, point_x, point_y, alpha), radius(radius) { }
   double radius;
};

struct surface : public shape
{
public:
   surface() { type = sh_surface; }
};

#endif