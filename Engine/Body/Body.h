#ifndef INCLUDE_BODY
#define INCLUDE_BODY

#include "Body\Shape.h"

class Body
{
public:
   shape* form;
   double mass;
   double iMass;
   double inert;// inertia tensor
   double iInert;

   Vector2 velocity;
   double angle_vel;

   Body(const shape* sh, double mazz, double velocity_x, double velocity_y, double angle_vel);
   Body(const Body& body);
   const Body& operator=(const Body& body);
   ~Body();

   Vector2 point_velocity(Vector2 point);
};

#endif