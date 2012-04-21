#ifndef INCLUDE_FRICTION_CONTRAINT
#define INCLUDE_FRICTION_CONTRAINT

#include "Constraint.h"
#include "Collision\Collision.h"

struct FrictionConstraintInit : ConstraintInit
{
   double impulseApplied;
};

class FrictionConstraint : public Constraint
{
public:
   FrictionConstraint(Collision* collision, world_vars* vars);
   void Init(const ConstraintInit* init);
   Vector2 _impulseDirection(void) const;
   double _deltaImpulse(void);
   size_t NumIter(void) const;
   bool Enough(void) const;
   double appliedNormalImpulse;

private:
   Vector2 norm;
   Vector2 tang;
   double vel_rel_n;
   double vel_rel_t;
   double roA;
   double roB;
   double min_lambda;
   double m_lambda;
   double sum_impulse;
   Collision* _collision;
};

#endif