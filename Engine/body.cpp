
#include "body.h"

Vector2 Body::point_velocity(Vector2 point)
{
   Vector2 r = point - form->point;
   return velocity + r * angle_vel;
}