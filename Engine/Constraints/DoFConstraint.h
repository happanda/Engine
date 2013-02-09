#ifndef INCLUDE_DOF_CONSTRAINT
#define INCLUDE_DOF_CONSTRAINT

#include "Constraint.h"

enum DoFType
{
    NONE     = 0,
    X_AXIS   = 1,
    Y_AXIS   = 2,
    XY_AXIS  = 3,
    ANGLE    = 4,
    X_ANGLE  = 5,
    Y_ANGLE  = 6,
    XY_ANGLE = 7
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
    void Fix();
    virtual void _deltaImpulse(Vector2& impulse, double& torque);
    virtual void init();

private:
    double x_init_;
    double y_init_;
    double angle_init_;
};

#endif