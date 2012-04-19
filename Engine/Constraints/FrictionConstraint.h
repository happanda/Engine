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

private:
   Vector2 tang;
   double vel_rel;
   Collision* _collision;
};

#endif