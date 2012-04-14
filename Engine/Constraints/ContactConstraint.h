#ifndef INCLUDE_CONTACT_CONTRAINT
#define INCLUDE_CONTACT_CONTRAINT

#include "Constraint.h"
#include "Collision\Collision.h"

class ContactConstraint : public Constraint
{
public:
   ContactConstraint(Collision* collision, world_vars* vars);
   void Init(Vector2 ForceExternal);
   Vector2 ImpulseDirection(void) const;
   double DeltaImpulse(void);
private:
   Collision* _collision;
   double _sum_impulse_n;
   double _sum_impulse_t;
};

#endif