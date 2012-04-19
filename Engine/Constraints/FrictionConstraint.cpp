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
   A = std::vector<std::vector<double>>(1);
   A[0] = std::vector<double>(1);
   Eta = std::vector<double>(1);
   Lambda = std::vector<double>(1, 1);
   impulseDirection = collision->normal.perpendicular();
   tang = -impulseDirection;
}

Vector2 FrictionConstraint::_impulseDirection(void) const
{
   return impulseDirection;
}

void FrictionConstraint::Init(Vector2 ForceExternal)
{
   const Body* bA = bodyA;
   const Body* bB = bodyB;
   double invMA = 1 / bA->mass;
   double invMB = 1 / bB->mass;
   double invIA = 1 / bA->inert;
   double invIB = 1 / bB->inert;

   Vector3 tang3(tang.v1, tang.v2, 0);
   Vector3 r3(rA.v1, rA.v2, 0);
   r3 = r3.cross(tang3);
   double ro1 = r3.v3;
   r3 = Vector3(rB.v1, rB.v2, 0);
   r3 = r3.cross(tang3);
   double ro2 = r3.v3;

   double tang2sq = tang.norm2sq();
   A[0][0] = (invMA + invMB) + ro1 * ro1 * invIA
      + ro2 * ro2 * invIB;

   vel_rel = tang * (bA->velocity - bB->velocity) + ro1 * bA->angle_vel
      - ro2 * bB->angle_vel;
   Eta[0] = vel_rel;

   // TODO: this operator must be present, but it leads to
   // enormous speeds after contact
   // Eta[0] = Eta[0] / ((double)w_vars->timeStep / 1000);
}

double FrictionConstraint::_deltaImpulse(void)
{
   // TODO: RESTITUTION here is wrong, min force must depend on relative velocity
   SolveLambda(A, Eta, Lambda, -w_vars->FRICTION, w_vars->FRICTION);
   impulse = Lambda[0] * ((double)w_vars->timeStep / 1000);
   return impulse;
}

size_t FrictionConstraint::NumIter(void) const
{
   return 200;
}
