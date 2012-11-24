#include <algorithm>
#include "Shape.h"
/*------------------- bbox -------------------*/
bbox::bbox(): left(0), right(0), top(0), bottom(0)
{ }
bbox::bbox(double left, double right, double top, double bottom):
left(left), right(right), top(top), bottom(bottom)
{ }
/*------------------ shape -------------------*/
shape::shape(): point(Vector2(0, 0)), alpha(0)
{ }
shape::shape(shape_type sh, double point_x, double point_y, double alpha):
type(sh), point(Vector2(point_x, point_y)), alpha(alpha)
{ }
shape::~shape()
{ }
void shape::rotate(double angle)
{
   alpha += angle;
}
/*---------------- rectangle -----------------*/
rectangle::rectangle(): shape(), h(1), w(1), half_diag(sqrt(2.0) / 2)
{ }
rectangle::rectangle(double point_x, double point_y, double alpha, double height, double width):
shape(sh_rectangle, point_x, point_y, alpha), h(height), w(width),
half_diag(sqrt(height * height + width * width) / 2)
{ }
rectangle::~rectangle()
{ }
// counter-clockwise left bottom, right bottom, etc.
void rectangle::get_points(Vector2& p1, Vector2& p2, Vector2& p3, Vector2& p4, double indent) const
{
   Vector2 v(h, w);
   double diag = v.norm2sq();// diagonal in rectangle
   double cosphi = h / (2 * diag);
   double sinphi = w / (2 * diag);

   Vector2 pnt(-h / 2 + indent, -w / 2 + indent);

   p1 = point + ::rotate(pnt, alpha);
   pnt.invX();
   p2 = point + ::rotate(pnt, alpha);
   pnt.invY();
   p3 = point + ::rotate(pnt, alpha);
   pnt.invX();
   p4 = point + ::rotate(pnt, alpha);
}
bbox rectangle::bounding_box() const
{
   Vector2 p1, p2, p3, p4;
   get_points(p1, p2, p3, p4, 0);
   double min_x = std::min(std::min(p1.v1, p2.v1), std::min(p3.v1, p4.v1));
   double max_x = std::max(std::max(p1.v1, p2.v1), std::max(p3.v1, p4.v1));
   double min_y = std::min(std::min(p1.v2, p2.v2), std::min(p3.v2, p4.v2));
   double max_y = std::max(std::max(p1.v2, p2.v2), std::max(p3.v2, p4.v2));
   return bbox(min_x, max_x, max_y, min_y);
   /*return bbox(point.v1 - half_diag, point.v1 + half_diag,
      point.v2 + half_diag, point.v2 - half_diag);*/
}
/*------------------ circle ------------------*/
circle::circle()
{
   type = sh_circle;
}
circle::circle(double point_x, double point_y, double alpha, double radius):
shape(sh_circle, point_x, point_y, alpha), radius(radius)
{ }
circle::~circle()
{ }
bbox circle::bounding_box() const
{
   return bbox(point.v1 - radius, point.v1 + radius, point.v2 + radius, point.v2 - radius);
}
