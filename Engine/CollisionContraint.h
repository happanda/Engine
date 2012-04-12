#ifndef INCLUDE_COLLISION_CONTRAINT
#define INCLUDE_COLLISION_CONTRAINT

#include "Constraint.h"
#include "collision.h"

class CollisionContraint : public Constraint
{
public:
   CollisionContraint(Collision* collision);
   void Init(Vector2 ForceExternal);
   Vector2 ImpulseDirection() const;
   double DeltaImpulse() const;
private:
   Collision* _collision;
   double _sum_impulse_n;
   double _sum_impulse_t;
};

#endif