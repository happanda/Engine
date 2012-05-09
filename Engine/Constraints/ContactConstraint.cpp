#include "ContactConstraint.h"
#include <float.h>
#include "Body\Body.h"
#include "Math\Vector2.h"
#include "Math\Vector3.h"
#include "World\World.h"
#include "Math\PGSsolver.h"

const double ContactConstraint::bias_factor = 0.5;
const double ContactConstraint::delta_slop = 0.0001;

ContactConstraint::ContactConstraint(const Collision* collision, size_t pnum, world_vars* vars):
Constraint(collision->body_one, collision->one[pnum] - collision->body_one->form->point,
           collision->body_two, collision->two[pnum] - collision->body_two->form->point,
           vars), _collision(collision)
{
    A = std::vector<std::vector<double>>(1);
    A[0] = std::vector<double>(1);
    Eta = std::vector<double>(1);
    Lambda = std::vector<double>(1, 1);
    //Impulse = _collision->normal;

    Vector2 norm = _collision->normal;
    vel_rel_n = norm * (bodyA->velocity - bodyB->velocity
        + Vector2(-rA.v2 * bodyA->angle_vel, rA.v1 * bodyA->angle_vel)
        - Vector2(-rB.v2 * bodyB->angle_vel, rB.v1 * bodyB->angle_vel));

    A[0][0] = 0;
    if (bodyA->mass < w_vars->UNMOVABLE_MASS)
    {
        A[0][0] += bodyA->iMass + norm * cross_cross(rA, norm) * bodyA->iInert;
    }
    if (bodyB->mass < w_vars->UNMOVABLE_MASS)
    {
        A[0][0] += bodyB->iMass + norm * cross_cross(rB, norm) * bodyB->iInert;
    }
    min_lambda = (1 + w_vars->RESTITUTION) / A[0][0] * w_vars->iTimeStep;
}

void ContactConstraint::init()
{
    Vector2 norm = _collision->normal;

    vel_rel_n = norm * (bodyA->velocity - bodyB->velocity
        + Vector2(-rA.v2 * bodyA->angle_vel, rA.v1 * bodyA->angle_vel)
        - Vector2(-rB.v2 * bodyB->angle_vel, rB.v1 * bodyB->angle_vel));
    Eta[0] = -vel_rel_n;

    // penetration correction impulse
    double v_bias = 0;
    double delta = (-bodyA->form->point - rA + bodyB->form->point + rB) * norm;
    if (delta > delta_slop)
        v_bias = bias_factor * (delta - delta_slop) * w_vars->iTimeStep;
    Eta[0] += v_bias;
    Eta[0] = Eta[0] * w_vars->iTimeStep;
    Eta[0] += Vector2(ForceExt[0] * bodyA->iMass,
        ForceExt[1] * bodyA->iMass) * norm;
    Eta[0] += ForceExt[2] * bodyA->iInert * roA;
    Eta[0] -= Vector2(ForceExt[3] * bodyB->iMass,
        ForceExt[4] * bodyB->iMass) * norm;
    Eta[0] -= ForceExt[5] * bodyB->iInert * roB;
}

void ContactConstraint::_deltaImpulse(Vector2& impulse, double& torque)
{
    init();
    double imp = 0;
    if (vel_rel_n < 0)
    {
        SolveLambda(A, Eta, Lambda, min_lambda * (-vel_rel_n), DBL_MAX);
        imp = Lambda[0] * w_vars->timeStep;
        if (imp + sum_impulse < 0)
            imp = -sum_impulse;
        sum_impulse += imp;
    }
    impulse = _collision->normal * imp;
}

size_t ContactConstraint::NumIter(void) const
{
    return 50;
}

bool ContactConstraint::Enough(void) const
{
    return vel_rel_n >= 0 || Constraint::Enough();
}