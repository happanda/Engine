#ifndef INCLUDE_GEOMETRY
#define INCLUDE_GEOMETRY

#include <math.h>
#include "LinearAlgebra.h"

class Segment
{
public:
   Vector2 head;
   Vector2 tail;
   Segment(): head(Vector2(0,0)), tail(Vector2(0,0)) { }
   Segment(Vector2 head, Vector2 tail): head(head), tail(tail) { }
   Segment(const Segment& segm): head(segm.head), tail(segm.tail) { }
   const Segment& operator=(const Segment& segm);
   inline double length()
   {
      Vector2 v = tail - head;
      return v.norm2();
   }
};

#endif