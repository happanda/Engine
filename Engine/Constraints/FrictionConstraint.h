#ifndef INCLUDE_FRICTION_CONTRAINT
#define INCLUDE_FRICTION_CONTRAINT

#include "Constraint.h"
#include "Collision\Collision.h"

class FrictionConstraint : public Constraint
{
public:
    FrictionConstraint(const Collision* collision, size_t pnum, world_vars* vars);
    size_t NumIter(void) const;
    bool Enough(void) const;
    double AppliedNormalImpulse;
protected:
    void _deltaImpulse(Vector2& impulse, double& torque);
private:
    void init();
    Vector2 norm;
    Vector2 tang;
    double vel_rel_n;
    double vel_rel_t;
    double roA;
    double roB;
    double min_lambda;
    double m_lambda;
    const Collision* _collision;
};

#endif