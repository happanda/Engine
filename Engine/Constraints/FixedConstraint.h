#ifndef INCLUDE_DOF_CONSTRAINT
#define INCLUDE_DOF_CONSTRAINT

#include "Constraint.h"

class FixedConstraint : public Constraint
{
public:
    FixedConstraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars);
    size_t NumIter(void) const;
    bool Enough(void) const;
protected:
    void _deltaImpulse(Vector2& impulse, double& torque);
    void init();
};

#endif