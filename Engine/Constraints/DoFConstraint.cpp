#include <float.h>
#include "DoFConstraint.h"
#include "Math\PGSsolver.h"
#include "World\World.h"

DoFConstraint::DoFConstraint(Body* body, DoFType type, world_vars* vars):
    Constraint(body, Vector2::ORIGIN, NULL, Vector2::ORIGIN, vars), dof_type(type)
{
    Type = DOF_CONSTRAINT;
    A = std::vector<std::vector<double>>(3);
    A[0] = std::vector<double>(3);
    A[1] = std::vector<double>(3);
    A[2] = std::vector<double>(3);
    Eta = std::vector<double>(3);
    Lambda = std::vector<double>(3, 1);
    //impulseDirection = _collision->normal;

    if ((type & X_AXIS) != 0)
    {
        x_init_ = body->form->point.v1;
        A[0][0] = bodyA->iMass;
    }
    if ((type & Y_AXIS) != 0)
    {
        y_init_ = body->form->point.v2;
        A[1][1] = bodyA->iMass;
    }
    if ((type & ANGLE) != 0)
    {
        angle_init_ = body->form->alpha;
        A[2][2] = bodyA->iInert;
    }
    //min_lambda = (1 + w_vars->RESTITUTION) / A[0][0] * w_vars->iTimeStep;
}

void DoFConstraint::ChangeConstraint(DoFType type)
{
    dof_type = type;
    if ((type & X_AXIS) != 0)
        A[0][0] = bodyA->iMass;
    if ((type & Y_AXIS) != 0)
        A[1][1] = bodyA->iMass;
    if ((type & ANGLE) != 0)
        A[2][2] = bodyA->iInert;
}

void DoFConstraint::init()
{
    //const DoFConstraintInit* dinit = static_cast<const DoFConstraintInit*>(init);
    if ((dof_type & X_AXIS) != 0)
        Eta[0] = -bodyA->velocity.v1;
    if ((dof_type & Y_AXIS) != 0)
        Eta[1] = -bodyA->velocity.v2;
    if ((dof_type & ANGLE) != 0)
        Eta[2] = -bodyA->angle_vel;
}

void DoFConstraint::Fix()
{
    if ((dof_type & X_AXIS) != 0)
    {
        bodyA->form->point.v1 = x_init_;
    }
    if ((dof_type & Y_AXIS) != 0)
    {
        bodyA->form->point.v2 = y_init_;
    }
    if ((dof_type & ANGLE) != 0)
    {
        bodyA->form->alpha = angle_init_;
    }
}

void DoFConstraint::_deltaImpulse(Vector2& impulse, double& torque)
{
    init();
    if (!Enough())
    {
        SolveLambda(A, Eta, Lambda, -DBL_MAX, DBL_MAX);
        impulse = Vector2(1, 0) * Lambda[0] + Vector2(0, 1) * Lambda[1];
        torque = Lambda[2];
        /*if (impulse + sum_impulse < 0)
        impulse = -sum_impulse;
        sum_impulse += impulse;*/
    }
    else
    {
        impulse = Vector2::ORIGIN;
    }
}

size_t DoFConstraint::NumIter(void) const
{
    return 50;
}

bool DoFConstraint::Enough(void) const
{
    double sum_vel = 0;
    if ((dof_type & X_AXIS) != 0)
        sum_vel += abs(bodyA->velocity.v1);
    if ((dof_type & Y_AXIS) != 0)
        sum_vel += abs(bodyA->velocity.v2);
    if ((dof_type & ANGLE) != 0)
        sum_vel += abs(bodyA->angle_vel);
    return sum_vel <= DBL_EPSILON;
}