
#include "Shape.h"

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
   /*Vector2 p[4];
   get_points(p[0], p[1], p[2], p[3], 0);
   double min_x = p[0].v1, max_x = p[0].v1,
      min_y = p[0].v2, max_y = p[0].v2;
   for (int i = 1; i < 4; i++)
   {
      if (p[i].v1 < min_x) min_x = p[i].v1;
      if (p[i].v1 > max_x) max_x = p[i].v1;
      if (p[i].v2 < min_y) min_y = p[i].v2;
      if (p[i].v2 > max_y) max_y = p[i].v2;
   }*/
   return bbox(point.v1 - half_diag, point.v1 + half_diag,
      point.v2 + half_diag, point.v2 - half_diag);
}

bbox circle::bounding_box()
{
   return bbox(point.v1 - radius, point.v1 + radius, point.v2 + radius, point.v2 - radius);
}