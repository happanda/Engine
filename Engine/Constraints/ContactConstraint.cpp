#include "ContactConstraint.h"
#include <float.h>
#include "Body\Body.h"
#include "Math\Vector2.h"
#include "Math\Vector3.h"
#include "World\World.h"
#include "Math\PGSsolver.h"

const double ContactConstraint::bias_factor = 0.3;
const double ContactConstraint::delta_slop = 0.0001;

ContactConstraint::ContactConstraint(Collision* collision, world_vars* vars):
Constraint(collision->body_one, collision->one[0] - collision->body_one->form->point,
           collision->body_two, collision->two[0] - collision->body_two->form->point,
           vars), _collision(collision)
{
   _sum_lambda = 0;
   A = std::vector<std::vector<double>>(1);
   A[0] = std::vector<double>(1);
   Eta = std::vector<double>(1);
   Lambda = std::vector<double>(1, 1);
   impulseDirection = _collision->normal;
}

Vector2 ContactConstraint::_impulseDirection(void) const
{
   return _collision->normal;
}

void ContactConstraint::Init(Vector2 ForceExternal)
{
   Vector2 norm = -_collision->normal;
   const Body* bA = bodyA;
   const Body* bB = bodyB;
   double invMA = 1 / bA->mass;
   double invMB = 1 / bB->mass;
   double invIA = 1 / bA->inert;
   double invIB = 1 / bB->inert;

   Vector3 norm3(norm.v1, norm.v2, 0);
   Vector3 r3(rA.v1, rA.v2, 0);
   r3 = r3.cross(norm3);
   double ro1 = r3.v3;
   r3 = Vector3(rB.v1, rB.v2, 0);
   r3 = r3.cross(norm3);
   double ro2 = r3.v3;

   double norm2sq = norm.norm2sq();
   A[0][0] = norm2sq * invMA + ro1 * ro1 * invIA
      + norm2sq * invMB + ro2 * ro2 * invIB;

   // penetration correction impulse
   double v_bias = 0;
   double delta = (bodyA->form->point + rA - bodyB->form->point - rB).norm2();
   if (delta > delta_slop)
      v_bias = bias_factor * (delta - delta_slop) / w_vars->timeStep;
   Eta[0] = norm * (bA->velocity - bB->velocity + norm * v_bias) + ro1 * bA->angle_vel
      - ro2 * bB->angle_vel;
   // TODO: this operator must be present, but it leads to
   // enormous speeds after contact
   //Eta[0] = Eta[0] / ((double)w_vars->timeStep / 1000);
}

double ContactConstraint::_deltaImpulse(void)
{   
   SolveLambda(A, Eta, Lambda, w_vars->RESTITUTION, DBL_MAX);
   double delta = (bodyA->form->point + rA - bodyB->form->point - rB).norm2();
   if (_sum_lambda + Lambda[0] < 0)
      Lambda[0] = -_sum_lambda;
   _sum_lambda += Lambda[0];
   impulse = Lambda[0] * ((double)w_vars->timeStep / 1000);
   return impulse;
}

size_t ContactConstraint::NumIter(void) const
{
   return 50;
}