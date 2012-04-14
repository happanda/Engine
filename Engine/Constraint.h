#ifndef INCLUDE_CONTRAINT
#define INCLUDE_CONTRAINT

#include <vector>
#include "Body.h"
#include "geometry.h"
#include "world.h"

class Constraint
{
public:
   Constraint(Body* bodyA, Body* bodyB, world_vars* vars);
   virtual void Init(Vector2 ForceExternal) = 0;
   virtual Vector2 ImpulseDirection(void) const = 0;
   virtual double DeltaImpulse(void) = 0;
   size_t NumIter(void) const;
   const Body* bodyA;
   const Body* bodyB;

   static const size_t MAX_ITER = 100;
protected:
   world_vars* w_vars;
   std::vector<std::vector<double>> Jacobian;
   std::vector<std::vector<double>> A;
   std::vector<double> Eta;
   std::vector<double> Lambda;
};

#endif