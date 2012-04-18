#ifndef INCLUDE_CONTACT_CONTRAINT
#define INCLUDE_CONTACT_CONTRAINT

#include "Constraint.h"
#include "Collision\Collision.h"

class ContactConstraint : public Constraint
{
public:
   ContactConstraint(Collision* collision, world_vars* vars);
   void Init(Vector2 ForceExternal);
   Vector2 _impulseDirection(void) const;
   double _deltaImpulse(void);
   size_t NumIter(void) const;

   static const double bias_factor;
   static const double delta_slop;
private:
   Collision* _collision;
   double _sum_lambda;
};

#endif