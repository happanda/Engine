#ifndef INCLUDE_SHAPE
#define INCLUDE_SHAPE

#include "Math\Geometry.h"

enum shape_type
{
   sh_rectangle,
   sh_circle,
   sh_surface
};

struct bbox
{
   double left;
   double right;
   double top;
   double bottom;
   bbox (): left(0), right(0), top(0), bottom(0) { }
   bbox (double left, double right, double top, double bottom):
      left(left), right(right), top(top), bottom(bottom) { }
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
   virtual ~shape() { }
   virtual bbox bounding_box() = 0;
};

struct rectangle : public shape
{
private:
   double half_diag;
public:
   double h, w;// sides lengths
   rectangle(): shape(), h(1), w(1), half_diag(sqrt(2.0) / 2) { }
   rectangle(double point_x, double point_y, double alpha,
      double height, double width): shape(sh_rectangle, point_x, point_y, alpha),
      h(height), w(width), half_diag(sqrt(height * height + width * width) / 2) { }
   void get_points(Vector2& p1, Vector2& p2, Vector2& p3, Vector2& p4, double indent) const;
   bbox bounding_box();
};

struct circle : public shape
{
public:
   circle() { type = sh_circle; }
   circle(double point_x, double point_y, double alpha, double radius):
      shape(sh_circle, point_x, point_y, alpha), radius(radius) { }
   double radius;
   bbox bounding_box();
};

struct surface : public shape
{
public:
   surface() { type = sh_surface; }
   bbox bounding_box() { }
};

#endif