#ifndef INCLUDE_CONTRAINT
#define INCLUDE_CONTRAINT

#include <vector>
#include "Body.h"
#include "geometry.h"

class Constraint
{
public:
   Constraint(Body* bodyA, Body* bodyB);
   virtual void Init(Vector2 ForceExternal) = 0;
   virtual Vector2 ImpulseDirection() const = 0;
   virtual double DeltaImpulse() const = 0;
   size_t NumIter() const;
   const Body* bodyA;
   const Body* bodyB;

   static const size_t MAX_ITER = 100;
protected:
   std::vector<std::vector<double>> Jacobian;
   std::vector<std::vector<double>> A;
   std::vector<double> Eta;
   std::vector<double> Lambda;
};

#endif