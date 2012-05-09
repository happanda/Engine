#include "FrictionConstraint.h"
#include <float.h>
#include "Body\Body.h"
#include "Math\Vector2.h"
#include "Math\Vector3.h"
#include "World\World.h"
#include "Math\PGSsolver.h"

FrictionConstraint::FrictionConstraint(const Collision* collision, size_t pnum, world_vars* vars):
Constraint(collision->body_one, collision->one[pnum] - collision->body_one->form->point,
           collision->body_two, collision->two[pnum] - collision->body_two->form->point,
           vars), _collision(collision)
{
    AppliedNormalImpulse = 0;
    A = std::vector<std::vector<double>>(1);
    A[0] = std::vector<double>(1);
    Eta = std::vector<double>(1);
    Lambda = std::vector<double>(1, 1);
    norm = _collision->normal;
    tang = norm.perpendicular();

    Vector2 vel = bodyA->velocity - bodyB->velocity
        + Vector2(-rA.v2 * bodyA->angle_vel, rA.v1 * bodyA->angle_vel)
        - Vector2(-rB.v2 * bodyB->angle_vel, rB.v1 * bodyB->angle_vel);
    vel_rel_n = norm * vel;
    vel_rel_t = tang * vel;

    A[0][0] = 0;
    if (bodyA->mass < w_vars->UNMOVABLE_MASS)
    {
        A[0][0] += bodyA->iMass + tang * cross_cross(rA, tang) * bodyA->iInert;
    }
    if (bodyB->mass < w_vars->UNMOVABLE_MASS)
    {
        A[0][0] += bodyB->iMass + tang * cross_cross(rB, tang) * bodyB->iInert;
    }
    min_lambda = w_vars->FRICTION / A[0][0] * w_vars->iTimeStep;
}

void FrictionConstraint::init()
{
    Vector2 vel = bodyA->velocity - bodyB->velocity
        + Vector2(-rA.v2 * bodyA->angle_vel, rA.v1 * bodyA->angle_vel)
        - Vector2(-rB.v2 * bodyB->angle_vel, rB.v1 * bodyB->angle_vel);
    vel_rel_n = norm * vel;
    vel_rel_t = tang * vel;
    Eta[0] = -vel_rel_t;
    Eta[0] = Eta[0] * w_vars->iTimeStep * w_vars->FRICTION;
}

void  FrictionConstraint::_deltaImpulse(Vector2& impulse, double& torque)
{
    init();
    double imp = 0;
    if (vel_rel_n < 0)
    {
        if (AppliedNormalImpulse > 0)
            m_lambda = min_lambda * AppliedNormalImpulse;
        else
            m_lambda = -min_lambda * vel_rel_t;
        //m_lambda = w_vars->FRICTION * (bodyA->mass + bodyB->mass) * 9.8;

        SolveLambda(A, Eta, Lambda, -m_lambda, m_lambda);
        imp = Lambda[0] * w_vars->timeStep;
        if (imp + sum_impulse < -w_vars->FRICTION * AppliedNormalImpulse)
            imp = -w_vars->FRICTION * AppliedNormalImpulse - sum_impulse;
        else if (imp + sum_impulse > w_vars->FRICTION * AppliedNormalImpulse)
            imp = w_vars->FRICTION * AppliedNormalImpulse - sum_impulse;
        sum_impulse += imp;
    }
    impulse = tang * imp;
}

size_t FrictionConstraint::NumIter(void) const
{
    return 50;
}

bool FrictionConstraint::Enough(void) const
{
    return vel_rel_n >= 0;
}