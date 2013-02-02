#ifndef INCLUDE_FIXED_CONSTRAINT
#define INCLUDE_FIXED_CONSTRAINT

#include "Constraint.h"

class FixedConstraint : public Constraint
{
public:
    FixedConstraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars);
    size_t NumIter(void) const;
    bool Enough(void) const;
protected:
    void Fix();
    void _deltaImpulse(Vector2& impulse, double& torque);
    void init();

private:
    double init_dist_;
    double rel_vel_;

    Vector2 rAorig;
    Vector2 rBorig;
};

#endif