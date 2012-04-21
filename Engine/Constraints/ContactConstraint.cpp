#include "ContactConstraint.h"
#include <float.h>
#include "Body\Body.h"
#include "Math\Vector2.h"
#include "Math\Vector3.h"
#include "World\World.h"
#include "Math\PGSsolver.h"

const double ContactConstraint::bias_factor = 0.5;
const double ContactConstraint::delta_slop = 0.0001;

ContactConstraint::ContactConstraint(Collision* collision, world_vars* vars):
Constraint(collision->body_one, collision->one[0] - collision->body_one->form->point,
           collision->body_two, collision->two[0] - collision->body_two->form->point,
           vars), _collision(collision)
{
   sum_impulse = 0;
   A = std::vector<std::vector<double>>(1);
   A[0] = std::vector<double>(1);
   Eta = std::vector<double>(1);
   Lambda = std::vector<double>(1, 1);
   impulseDirection = _collision->normal;

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

void ContactConstraint::Init(const ConstraintInit* init)
{
   Vector2 norm = _collision->normal;

   vel_rel_n = norm * (bodyA->velocity - bodyB->velocity
      + Vector2(-rA.v2 * bodyA->angle_vel, rA.v1 * bodyA->angle_vel)
      - Vector2(-rB.v2 * bodyB->angle_vel, rB.v1 * bodyB->angle_vel));
   Eta[0] = -vel_rel_n;

   // penetration correction impulse
   double v_bias = 0;
   double delta = (bodyA->form->point + rA - bodyB->form->point - rB) * norm;
   if (delta > delta_slop)
      v_bias = bias_factor * (delta - delta_slop) * w_vars->iTimeStep;
   Eta[0] += v_bias;
   Eta[0] = Eta[0] * w_vars->iTimeStep;
}

double ContactConstraint::_deltaImpulse(void)
{
   if (vel_rel_n < 0)
   {
      SolveLambda(A, Eta, Lambda, min_lambda * (-vel_rel_n), DBL_MAX);
      impulse = Lambda[0] * w_vars->timeStep;
      if (impulse + sum_impulse < 0)
         impulse = -sum_impulse;
      sum_impulse += impulse;
   }
   else
   {
      impulse = 0;
   }
   if (Lambda[0] > 10000)
   {
      printf("To the moon!!\n");
   }
   return impulse;
}

Vector2 ContactConstraint::_impulseDirection(void) const
{
   return impulseDirection;
}

size_t ContactConstraint::NumIter(void) const
{
   return 50;
}

bool ContactConstraint::Enough(void) const
{
   return vel_rel_n >= 0;
}