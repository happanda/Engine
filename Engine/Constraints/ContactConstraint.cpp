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
   Lambda = std::vector<double>(1, 1);
}

Vector2 ContactConstraint::ImpulseDirection(void) const
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
   double norm2sq = norm.norm2sq();
   A[0][0] = norm2sq * invMA + ro1 * ro1 * invIA
      + norm2sq * invMB + ro2 * ro2 * invIB;

   Eta = std::vector<double>(1);
   Jacobian = vector<vector<double>>(1);
   Jacobian[0] = vector<double>(1);
   Eta[0] = (norm * bA->velocity) + ro1 * bA->angle_vel
      - (norm * bB->velocity) - ro2 * bB->angle_vel;
   // TODO: this operator must be present, but it leads to
   // enormous speeds after contact
   // Eta[0] = Eta[0] / ((double)w_vars->timeStep / 1000);
}

double ContactConstraint::DeltaImpulse(void)
{   
   SolveLambda(A, Eta, Lambda, w_vars->RESTITUTION, DBL_MAX);
   return Lambda[0] *  ((double)w_vars->timeStep / 1000);
}

size_t ContactConstraint::NumIter(void) const
{
   return 20;
}