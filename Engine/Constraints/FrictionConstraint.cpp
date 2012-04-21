#include "FrictionConstraint.h"
#include <float.h>
#include "Body\Body.h"
#include "Math\Vector2.h"
#include "Math\Vector3.h"
#include "World\World.h"
#include "Math\PGSsolver.h"

FrictionConstraint::FrictionConstraint(Collision* collision, world_vars* vars):
Constraint(collision->body_one, collision->one[0] - collision->body_one->form->point,
           collision->body_two, collision->two[0] - collision->body_two->form->point,
           vars), _collision(collision)
{
   sum_impulse = 0;
   appliedNormalImpulse = 0;
   A = std::vector<std::vector<double>>(1);
   A[0] = std::vector<double>(1);
   Eta = std::vector<double>(1);
   Lambda = std::vector<double>(1, 1);
   norm = _collision->normal;
   impulseDirection = norm.perpendicular();
   tang = impulseDirection;

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

void FrictionConstraint::Init(const ConstraintInit* init)
{
   Vector2 vel = bodyA->velocity - bodyB->velocity
      + Vector2(-rA.v2 * bodyA->angle_vel, rA.v1 * bodyA->angle_vel)
      - Vector2(-rB.v2 * bodyB->angle_vel, rB.v1 * bodyB->angle_vel);
   vel_rel_n = norm * vel;
   vel_rel_t = tang * vel;
   Eta[0] = -vel_rel_t;
   Eta[0] = Eta[0] * w_vars->iTimeStep * w_vars->FRICTION;
   appliedNormalImpulse = 0;
}

double FrictionConstraint::_deltaImpulse(void)
{
   if (vel_rel_n < 0)
   {
      if (appliedNormalImpulse > 0)
         m_lambda = min_lambda * appliedNormalImpulse;
      else
         m_lambda = -min_lambda * vel_rel_t;
      //m_lambda = w_vars->FRICTION * (bodyA->mass + bodyB->mass) * 9.8;

      SolveLambda(A, Eta, Lambda, -m_lambda, m_lambda);
      impulse = Lambda[0] * w_vars->timeStep;
      if (impulse + sum_impulse < 0)
         impulse = -sum_impulse;
      sum_impulse += impulse;
   }
   else
   {
      impulse = 0;
   }
   return impulse;
}

Vector2 FrictionConstraint::_impulseDirection(void) const
{
   return impulseDirection;
}

size_t FrictionConstraint::NumIter(void) const
{
   return 50;
}

bool FrictionConstraint::Enough(void) const
{
   return vel_rel_n >= 0;
}