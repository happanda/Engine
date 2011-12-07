
#include "math_routines.h"
#include <float.h>
#include "geometry.h"
#include "gjk.h"

void get_points(rectangle rect, Vector2& p1, Vector2& p2,
                 Vector2& p3, Vector2& p4, double indent)
{
   Vector2 v(rect.h, rect.w);
   double diag = v.norm2sq();// diagonal in rectangle
   double cosphi = rect.h / (2 * diag);
   double sinphi = rect.w / (2 * diag);
   
   Vector2 point(-rect.h / 2 + indent, -rect.w / 2 + indent);

   p1 = rect.point + rotate(point, rect.alpha);
   point.invX();
   p2 = rect.point + rotate(point, rect.alpha);
   point.invY();
   p3 = rect.point + rotate(point, rect.alpha);
   point.invX();
   p4 = rect.point + rotate(point, rect.alpha);
}

// computes a vector V, such that (V * codirect > 0)
Vector2 perpendicular(Vector2 vect, Vector2 codirect)
{
   Vector2 perp = vect.perpendicular();
   if (perp * codirect < 0)
      perp = -perp;
   return perp;
}

bool isOn(Vector2 point, Vector2 a, Vector2 b)
{
   if (point.v1 < a.v1 && point.v1 < b.v1)
      return false;
   if (point.v2 < a.v2 && point.v2 < b.v2)
      return false;
   if (abs((b.v2 - a.v2) * (point.v1 - a.v1) - (b.v1 - a.v1) * (point.v2 - a.v2))
      > DBL_EPSILON)
      return false;
   return true;
}

double project(Vector2 vect, Vector2 line)
{
   line.normalize2();
   return (vect * line);
}

Vector2 closest_point(Vector2 point, Segment segment)
{
   Vector2 AB = segment.tail - segment.head;
   Vector2 AP = point - segment.head;
   if (AB * AP <= 0)// closest is A
      return segment.head;
   Vector2 BA = -AB;
   Vector2 BP = point - segment.tail;
   if (BA * BP <= 0)// closest is B
      return segment.tail;
   // closest if somewhere inside segment
   // project P onto segment
   AB.normalize2();
   return segment.head + AB * (AP * AB);
}

double distance(Vector2 point, Segment segment)
{
   Vector2 proj = closest_point(point, segment);
   return (proj - point).norm2();
}
