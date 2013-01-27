#include <iostream>
#include <float.h>
#include "FixedConstraint.h"
#include "Math\PGSsolver.h"
#include "World\World.h"

FixedConstraint::FixedConstraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars)
    : Constraint(bodyA, rA, bodyB, rB, vars)
{
    Type = FIXED_CONSTRAINT;
    A = std::vector<std::vector<double>>(1);
    A[0] = std::vector<double>(1);
    Eta = std::vector<double>(1);
    Lambda = std::vector<double>(1, 1);

    Vector2 dist = (bodyB->form->point + rB - bodyA->form->point - rA);
    init_dist_ = dist.norm2();
    dist.normalize2();

    /************************************************************************/
    /* http://www.codezealot.org/archives/267                               */
    /************************************************************************/
}

void FixedConstraint::init()
{
    Vector2 dist = (bodyB->form->point + rB - bodyA->form->point - rA);
    dist.normalize2();

    Vector3 dist3(dist.v1, dist.v2, 0);
    Vector3 rA3(rA.v1, rA.v2, 0);
    Vector3 rB3(rB.v1, rB.v2, 0);

    A[0][0] = (bodyA->iMass + bodyB->iMass)
        + bodyA->iInert * (dist3.cross(rA3).v3) * (dist3.cross(rA3).v3)
        + bodyB->iInert * (dist3.cross(rB3).v3) * (dist3.cross(rB3).v3);

    rel_vel_ = dist * (bodyB->point_velocity(bodyB->form->point + rB)
        - bodyA->point_velocity(bodyA->form->point + rA));
    Eta[0] = -rel_vel_;
}

void FixedConstraint::_deltaImpulse(Vector2& impulse, double& torque)
{
    init();
    torque = 0;

    if (!Enough())
    {
        SolveLambda(A, Eta, Lambda, -DBL_MAX, DBL_MAX);
        Vector2 dist = -(bodyB->form->point + rB - bodyA->form->point - rA);
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
    Vector2 dist = bodyB->form->point + rB - bodyA->form->point - rA;

    double delta = dist.norm2() - init_dist_;
    dist.normalize2();

    if (abs(delta) > DBL_EPSILON)
    {
        dist.normalize2();

        Vector2 fix_dist = dist * delta;

        bodyA->form->point = bodyA->form->point + fix_dist * bodyA->iMass * 0.5;
        bodyB->form->point = bodyB->form->point - fix_dist * bodyB->iMass * 0.5;

        Vector3 dist3(fix_dist.v1, fix_dist.v2, 0);
        Vector3 rA3(rA.v1, rA.v2, 0);
        Vector3 rB3(rB.v1, rB.v2, 0);

        bodyA->form->alpha += rA3.cross(dist3).v3 * bodyA->iInert;
        bodyB->form->alpha += rB3.cross(dist3).v3 * bodyB->iInert;
    }
}

size_t FixedConstraint::NumIter(void) const
{
    return 500;
}

bool FixedConstraint::Enough(void) const
{
    Vector2 deltaP = bodyB->form->point + rB - bodyA->form->point - rA;
    deltaP.normalize2();
    double rel_vel = deltaP * (bodyB->point_velocity(bodyB->form->point + rB)
        - bodyA->point_velocity(bodyA->form->point + rA));
    return abs(rel_vel) < DBL_EPSILON || Constraint::Enough();
}
