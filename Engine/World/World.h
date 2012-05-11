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
   void addBody(Body body);
   void addConstraint(Constraint* constraint);
   void update(double deltaT);
   void apply_forces(double deltaT);
   void resolve_collision(double deltaT);
   void resolve_constraints(double deltaT);

   std::vector<Collision> collisions;
   std::vector<Body> bodies;
   std::vector<Constraint*> constraints;
   world_vars vars;

   void resolve_collision_deprecated(double deltaT);
private:
    double force_ext[6];
};

#endif