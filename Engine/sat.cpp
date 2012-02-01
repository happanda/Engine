
#include "sat.h"
#include <float.h>
#include "math_routines.h"

int check_collide(const rectangle& rectA, const rectangle& rectB,
                  Vector2& point, Segment& edge)
{
   double minIntersect;
   int shapeNum = -1;
   double intersect = find_min_intersection(rectA, rectB, point, edge);
   if (intersect < 0)
      return -1;
   else
   {
      minIntersect = intersect;
      shapeNum = 0;
   }

   Vector2 p;
   Segment e;
   intersect = find_min_intersection(rectB, rectA, p, e);
   if (intersect < 0)
      return -1;
   else
   {
      if (intersect < minIntersect)
      {
         minIntersect = intersect;
         shapeNum = 1;
         point = p;
         edge = e;
      }
   }
   return shapeNum;
}

double find_min_intersection(const rectangle& rectA, const rectangle& rectB,
                   Vector2& point, Segment& edge)
{
   Vector2 pA[4];
   Vector2 pB[4];
   rectA.get_points(pA[0], pA[1], pA[2], pA[3], 0);
   rectB.get_points(pB[0], pB[1], pB[2], pB[3], 0);

   double minProjIntersect = DBL_MAX;// minimum intersection of projections

   for (int ln = 0; ln < 4; ln++)
   {
      Vector2 line = (pA[(ln+1) % 4] - pA[ln]).perpendicular();

      double maxA = project(pA[ln], line),
         minA = project(pA[(ln+2) % 4], line),
         maxB = project(pB[0], line);
      double minB = maxB;
      short minIdxB = 0;

      // compute intervals of projection of B onto the line
      for (int pn = 0; pn < 4; pn++)
      {
         double projB = project(pB[pn], line);
         if (projB > maxB)
            maxB = projB;
         else if (projB < minB)
         {
            minB = projB;
            minIdxB = pn;
         }
      }
      // check collision length
      if (minA < minB)
      {
         if (maxA < minB)
            return -1;// found separating axis
         else if (maxA - minB < minProjIntersect)
         {
            minProjIntersect = maxA - minB;
            point = pB[minIdxB];
            edge = Segment(pA[(ln+1) % 4], pA[ln]);
         }
      }
   }
   return minProjIntersect;
}

int check_collide(const rectangle& rect, const circle& circ,
                   Vector2& point, Segment& edge)
{
   return 0;
}