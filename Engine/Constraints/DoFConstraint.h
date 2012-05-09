#ifndef INCLUDE_DOF_CONSTRAINT
#define INCLUDE_DOF_CONSTRAINT

#include "Constraint.h"

enum DoFType
{
    X_AXIS = 0x001,
    Y_AXIS = 0x010,
    ANGLE =  0x100,
    XY_AXIS = 0x011,
    X_ANGLE = 0x101,
    Y_ANGLE = 0x110,
    XY_ANGLE = 0x111
};

class DoFConstraint : public Constraint
{
public:
    DoFConstraint(Body* body, DoFType type, world_vars* vars);
    size_t NumIter(void) const;
    bool Enough(void) const;
    DoFType dof_type;
protected:
    void _deltaImpulse(Vector2& impulse, double& torque);
private:
    void init();
};

#endif