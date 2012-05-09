#ifndef INCLUDE_CONTACT_CONTRAINT
#define INCLUDE_CONTACT_CONTRAINT

#include "Constraint.h"
#include "Collision\Collision.h"

class ContactConstraint : public Constraint
{
public:
    ContactConstraint(const Collision* collision, size_t pnum, world_vars* vars);
    size_t NumIter(void) const;
    bool Enough(void) const;

    static const double bias_factor;
    static const double delta_slop;
protected:
    void _deltaImpulse(Vector2& impulse, double& torque);
private:
    void init();
    double vel_rel_n;
    double roA;
    double roB;
    double min_lambda;
    const Collision* _collision;
};

#endif