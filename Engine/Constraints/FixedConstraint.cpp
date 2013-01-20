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

    Vector2 dist = (bodyB->form->point - bodyA->form->point);
    init_dist_ = dist.norm2();
    dist.normalize2();

    /************************************************************************/
    /* http://www.codezealot.org/archives/267                               */
    /************************************************************************/
    A[0][0] = (bodyA->iMass + bodyB->iMass)
        + bodyA->iInert * (dist * bodyA->form->point) * (dist * bodyA->form->point)
        + bodyB->iInert * (dist * bodyB->form->point) * (dist * bodyB->form->point);
}

void FixedConstraint::init()
{
    Vector2 dist = (bodyB->form->point - bodyA->form->point);
    dist.normalize2();

    A[0][0] = (bodyA->iMass + bodyB->iMass);
        //+ bodyA->iInert * (dist * bodyA->form->point) * (dist * bodyA->form->point)
        //+ bodyB->iInert * (dist * bodyB->form->point) * (dist * bodyB->form->point);

    rel_vel_ = dist * (bodyB->velocity - bodyA->velocity);
    Eta[0] = -rel_vel_;
}

void FixedConstraint::_deltaImpulse(Vector2& impulse, double& torque)
{
    init();
    torque = 0;

    if (!Enough())
    {
        SolveLambda(A, Eta, Lambda, -DBL_MAX, DBL_MAX);
        Vector2 dist = -(bodyB->form->point - bodyA->form->point);
        dist.normalize2();
        impulse = dist * Lambda[0] * 0.5;
    }
    else
    {
        impulse = Vector2::ORIGIN;
    }
}

void FixedConstraint::Fix()
{
    Vector2 dist = bodyB->form->point - bodyA->form->point;

    double delta = dist.norm2() - init_dist_;
    if (abs(delta) > DBL_EPSILON)
    {
        dist.normalize2();

        double fix_d = A[0][0] * delta;
        Vector2 fix_dist = dist * fix_d;

        bodyA->form->point = bodyA->form->point + fix_dist * bodyA->iMass;
        bodyB->form->point = bodyB->form->point + fix_dist * bodyB->iMass;
    }
}

size_t FixedConstraint::NumIter(void) const
{
    return 500;
}

bool FixedConstraint::Enough(void) const
{
    Vector2 deltaP = bodyB->form->point - bodyA->form->point;
    deltaP.normalize2();
    double rel_vel = deltaP * (bodyB->velocity - bodyA->velocity);
    return abs(rel_vel) < DBL_EPSILON || Constraint::Enough();
}