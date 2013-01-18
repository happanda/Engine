#ifndef INCLUDE_CONTRAINT
#define INCLUDE_CONTRAINT

#include <vector>
#include "Body\Body.h"
#include "Math\Geometry.h"
#include "World\WorldVars.h"

enum ConstraintType
{
    BASE_CONSTRAINT,
    CONTACT_CONSTRAINT,
    FRICTION_CONSTRAINT,
    DOF_CONSTRAINT,
    FIXED_CONSTRAINT
};

struct ConstraintInit
{
    double force_ext[6];
};

class Constraint
{
public:
    Constraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars);
    Constraint(const Constraint& constr);
    const Constraint& operator=(const Constraint& constr);

    void DeltaImpulse();
    void ApplyImpulse();
    virtual void Fix();
    virtual bool Enough(void) const;
    virtual size_t NumIter(void) const;

    Body* bodyA;
    Body* bodyB;
    const Vector2 rA;
    const Vector2 rB;

    void SetForceExt(double* force);
    double ForceExt[6];
    Vector2 Impulse;
    double Torque;
    const world_vars* w_vars;

    ConstraintType Type;
    static const size_t MAX_ITER = 50;
protected:
    size_t iterCount;
    double sum_impulse;

    virtual void _deltaImpulse(Vector2& impulse, double& torque) = 0;

    std::vector<std::vector<double>> Jacobian;
    std::vector<std::vector<double>> A;
    std::vector<double> Eta;
    std::vector<double> Lambda;
};

#endif