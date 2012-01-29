#ifndef INCLUDE_SHAPE
#define INCLUDE_SHAPE

#include "geometry.h"

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
   virtual bbox bounding_box() = 0;
};

struct rectangle : public shape
{
public:
   double h, w;// sides lengths
   rectangle(): shape(), h(1), w(1) { }
   rectangle(double point_x, double point_y, double alpha,
      double height, double width): shape(sh_rectangle, point_x, point_y, alpha),
      h(height), w(width) { }
   void get_points(Vector2& p1, Vector2& p2, Vector2& p3, Vector2& p4, double indent)
   {
      Vector2 v(h, w);
      double diag = v.norm2sq();// diagonal in rectangle
      double cosphi = h / (2 * diag);
      double sinphi = w / (2 * diag);

      Vector2 point(-h / 2 + indent, -w / 2 + indent);

      p1 = point + ::rotate(point, alpha);
      point.invX();
      p2 = point + ::rotate(point, alpha);
      point.invY();
      p3 = point + ::rotate(point, alpha);
      point.invX();
      p4 = point + ::rotate(point, alpha);
   }
   bbox bounding_box()
   {
      Vector2 p[4];
      get_points(p[0], p[1], p[2], p[3], 0);
      double min_x = p[0].v1, max_x = p[0].v1,
         min_y = p[0].v2, max_y = p[0].v2;
      for (int i = 1; i < 4; i++)
      {
         if (p[i].v1 < min_x) min_x = p[i].v1;
         if (p[i].v1 > max_x) max_x = p[i].v1;
         if (p[i].v2 < min_y) min_y = p[i].v2;
         if (p[i].v2 > max_y) max_y = p[i].v2;
      }
      return bbox(min_x, max_x, max_y, min_y);
   }
};

struct circle : public shape
{
public:
   circle() { type = sh_circle; }
   circle(double point_x, double point_y, double alpha, double radius):
      shape(sh_circle, point_x, point_y, alpha), radius(radius) { }
   double radius;
   bbox bounding_box()
   { return bbox(point.v1 - radius, point.v1 + radius, point.v2 - radius, point.v2 + radius); }
};

struct surface : public shape
{
public:
   surface() { type = sh_surface; }
   bbox bounding_box() { }
};

#endif