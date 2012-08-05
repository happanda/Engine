
#include "Shape.h"

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

bbox rectangle::bounding_box()
{
   return bbox(point.v1 - half_diag, point.v1 + half_diag,
      point.v2 + half_diag, point.v2 - half_diag);
}

bbox circle::bounding_box()
{
   return bbox(point.v1 - radius, point.v1 + radius, point.v2 + radius, point.v2 - radius);
}