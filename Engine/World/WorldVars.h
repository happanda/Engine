#ifndef INCLUDE_WORLDVARS
#define INCLUDE_WORLDVARS

#include "Math\Vector2.h"

struct world_vars
{
   double RESTITUTION;
   double FRICTION;
   Vector2 GRAVITATION;
   double UNMOVABLE_MASS;
   // timestep in seconds
   double timeStep;
   double iTimeStep;
};

#endif