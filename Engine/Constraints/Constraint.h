#ifndef INCLUDE_CONTRAINT
#define INCLUDE_CONTRAINT

#include <vector>
#include "Body\Body.h"
#include "Math\Geometry.h"
#include "World\World.h"

class Constraint
{
public:
   Constraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars);
   virtual void Init(Vector2 ForceExternal) = 0;
   void DeltaImpulse();
   void ApplyImpulse();
   size_t NumIter(void) const;

   Body* bodyA;
   Body* bodyB;
   double impulse;
   Vector2 impulseDirection;
   const Vector2 rA;
   const Vector2 rB;
   world_vars* w_vars;

   static const size_t MAX_ITER = 100;
protected:
   virtual Vector2 _impulseDirection(void) const = 0;
   virtual double _deltaImpulse(void) = 0;
   std::vector<std::vector<double>> Jacobian;
   std::vector<std::vector<double>> A;
   std::vector<double> Eta;
   std::vector<double> Lambda;
};

#endif