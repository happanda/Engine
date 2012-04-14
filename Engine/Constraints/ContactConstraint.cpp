#include "ContactConstraint.h"
#include <float.h>
#include "Body\Body.h"
#include "Math\Vector2.h"
#include "Math\Vector3.h"
#include "World\World.h"
#include "Math\PGSsolver.h"

ContactConstraint::ContactConstraint(Collision* collision, world_vars* vars):
Constraint(collision->body_one, collision->body_two, vars), _collision(collision)
{
   _sum_impulse_n = 0;
   _sum_impulse_t = 0;
   Vector2 norm = _collision->normal;
   Body* bA = _collision->body_one;
   Body* bB = _collision->body_two;
   double invMA = 1 / bA->mass;
   double invMB = 1 / bB->mass;
   double invIA = 1 / bA->inert;
   double invIB = 1 / bB->inert;
   Vector2 r1 = _collision->one[0] - bA->form->point;
   Vector2 r2 = _collision->two[0] - bB->form->point;
   Vector3 norm3(norm.v1, norm.v2, 0);
   Vector3 r3(r1.v1, r1.v2, 0);
   r3.cross(norm3);
   double ro1 = r3.v3;
   r3 = Vector3(r2.v1, r2.v2, 0);
   r3.cross(norm3);
   double ro2 = r3.v3;

   A = std::vector<std::vector<double>>(1);
   A[0] = std::vector<double>(1);
   double norm1sq = norm.v1 * norm.v1;
   double norm2sq = norm.v2 * norm.v2;
   A[0][0] = - norm1sq * invMA - norm2sq * invMA - ro1 * invIA
      + norm1sq * invMB + norm2sq * invMB + ro2 * invIB;

   Eta = std::vector<double>(1);
   Eta[0] = (norm * bA->velocity) - (norm * bB->velocity)
      + ro1 * bA->angle_vel - ro2 * bB->angle_vel;
   Eta[0] /= w_vars->timeStep;
   Lambda = std::vector<double>(1, 1);
}

Vector2 ContactConstraint::ImpulseDirection(void) const
{
   return _collision->normal;
}

void ContactConstraint::Init(Vector2 ForceExternal)
{
}

double ContactConstraint::DeltaImpulse(void)
{   
   SolveLambda(A, Eta, Lambda, 0, DBL_MAX);
   return Lambda[0] * w_vars->timeStep;
}

size_t ContactConstraint::NumIter(void) const
{
   return 20;
}