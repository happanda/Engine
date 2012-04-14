
#include "Math\Geometry.h"


const Segment& Segment::operator=(const Segment& segm)
{
   head = segm.head;
   tail = segm.tail;
   return *this;
}