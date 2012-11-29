#include <iostream>
#include <float.h>
#include "FixedConstraint.h"
#include "Math\PGSsolver.h"
#include "World\World.h"

FixedConstraint::FixedConstraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars)
    : Constraint(bodyA, Vector2::ORIGIN, bodyB, Vector2::ORIGIN, vars)
{
    Type = FIXED_CONSTRAINT;
    A = std::vector<std::vector<double>>(1);
    A[0] = std::vector<double>(1);
    Eta = std::vector<double>(1);
    Lambda = std::vector<double>(1, 1);
    //impulseDirection = _collision->normal;

    A[0][0] = (bodyB->form->point - bodyA->form->point).norm2sq() * (bodyA->iMass + bodyB->iMass);
    //min_lambda = (1 + w_vars->RESTITUTION) / A[0][0] * w_vars->iTimeStep;
}

void FixedConstraint::init()
{
    Vector2 deltaP = bodyB->form->point - bodyA->form->point;
    deltaP.normalize2();
    double rel_vel = deltaP * (bodyA->velocity - bodyB->velocity);
    Eta[0] = rel_vel * w_vars->iTimeStep;
}

void FixedConstraint::_deltaImpulse(Vector2& impulse, double& torque)
{
    init();
    torque = 0;

    if (!Enough())
    {
        SolveLambda(A, Eta, Lambda, -DBL_MAX, DBL_MAX);
        Vector2 dist = -bodyB->form->point + bodyA->form->point;
        dist.normalize2();
        impulse = dist * Lambda[0] * 0.5;
    }
    else
    {
        impulse = Vector2::ORIGIN;
    }
}

size_t FixedConstraint::NumIter(void) const
{
    return 5;
}

bool FixedConstraint::Enough(void) const
{
    return Constraint::Enough();
}