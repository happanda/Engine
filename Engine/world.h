#ifndef INCLUDE_WORLD
#define INCLUDE_WORLD

#include <vector>
#include "body.h"
#include "collision.h"

class World
{
public:
   World();
   void init();
   void addBody(Body body) { bodies.push_back(body); }
   void update(double deltaT);
   void resolve_collision();
   void apply_forces(double deltaT);

   std::vector<Collision> collisions;
   std::vector<Body> bodies;
   double RESTITUTION;
   double FRICTION;
   Vector2 GRAVITATION;
   double UNMOVABLE_MASS;
   // timestep in milliseconds
   int timeStep;
};

#endif