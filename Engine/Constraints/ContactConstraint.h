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
   bool Enough(void) const;

   static const double bias_factor;
   static const double delta_slop;
private:
   double vel_rel;
   double roA;
   double roB;
   double min_lambda;
   double sum_impulse;
   Collision* _collision;
};

#endif