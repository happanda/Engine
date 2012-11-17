#ifndef INCLUDE_DOF_CONSTRAINT
#define INCLUDE_DOF_CONSTRAINT

#include "Constraint.h"

enum DoFType
{
    NONE     = 0x000,
    X_AXIS   = 0x001,
    Y_AXIS   = 0x002,
    XY_AXIS  = 0x003,
    ANGLE    = 0x004,
    X_ANGLE  = 0x005,
    Y_ANGLE  = 0x006,
    XY_ANGLE = 0x007
};

class DoFConstraint : public Constraint
{
public:
    DoFConstraint(Body* body, DoFType type, world_vars* vars);
    void ChangeConstraint(DoFType type);
    size_t NumIter(void) const;
    bool Enough(void) const;
    DoFType dof_type;
protected:
    void _deltaImpulse(Vector2& impulse, double& torque);
    void init();
};

#endif