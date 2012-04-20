#ifndef INCLUDE_FRICTION_CONTRAINT
#define INCLUDE_FRICTION_CONTRAINT

#include "Constraint.h"
#include "Collision\Collision.h"

class FrictionConstraint : public Constraint
{
public:
   FrictionConstraint(Collision* collision, world_vars* vars);
   void Init(Vector2 ForceExternal);
   Vector2 _impulseDirection(void) const;
   double _deltaImpulse(void);
   size_t NumIter(void) const;
   bool Enough(void) const;

private:
   Vector2 norm;
   Vector2 tang;
   double vel_rel;
   double vel_rel_t;
   double roA;
   double roB;
   double min_lambda;
   double sum_impulse;
   Collision* _collision;
};

#endif