
#include "Body.h"

Vector2 Body::point_velocity(Vector2 point)
{
   Vector2 r = point - form->point;
   return velocity + Vector2(-r.v2 * angle_vel, r.v1 * angle_vel);
}