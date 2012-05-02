#ifndef INCLUDE_WORLD
#define INCLUDE_WORLD

#include <vector>
#include "WorldVars.h"
#include "Body\Body.h"
#include "Collision\Collision.h"
#include "Constraints\Constraint.h"

class World
{
public:
   World();
   void init();
   void addBody(Body body) { bodies.push_back(body); }
   void addConstraint(Constraint* constraint) { constraints.push_back(constraint); }
   void update(double deltaT);
   void resolve_collision_old(double deltaT);
   void resolve_collision(double deltaT);
   void resolve_constraints(double deltaT);
   void apply_forces(double deltaT);

   std::vector<Collision> collisions;
   std::vector<Body> bodies;
   std::vector<Constraint*> constraints;
   world_vars vars;
};

#endif