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
   A = std::vector<std::vector<double>>(1);
   A[0] = std::vector<double>(1);
   Eta = std::vector<double>(1);
   Lambda = std::vector<double>(1, 1);
   impulseDirection = _collision->normal;
}

Vector2 ContactConstraint::_impulseDirection(void) const
{
   return impulseDirection;
}

void ContactConstraint::Init(Vector2 ForceExternal)
{
   Vector2 norm = -_collision->normal;
   const Body* bA = bodyA;
   const Body* bB = bodyB;
   double invMA = bA->iMass;
   double invMB = bB->iMass;
   double invIA = bA->iInert;
   double invIB = bB->iInert;

   Vector3 norm3(norm.v1, norm.v2, 0);
   Vector3 r3(rA.v1, rA.v2, 0);
   r3 = r3.cross(norm3);
   double ro1 = r3.v3;
   r3 = Vector3(rB.v1, rB.v2, 0);
   r3 = r3.cross(norm3);
   double ro2 = r3.v3;

   double norm2sq = norm.norm2sq();
   A[0][0] = (invMA + invMB) + ro1 * ro1 * invIA
      + ro2 * ro2 * invIB;

   vel_rel = norm * (bA->velocity - bB->velocity) + ro1 * bA->angle_vel
      - ro2 * bB->angle_vel;
   Eta[0] = vel_rel;

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
   // TODO: RESTITUTION here is wrong, min force must depend on relative velocity
   SolveLambda(A, Eta, Lambda, w_vars->RESTITUTION, DBL_MAX);
   if (Lambda[0] > 1000)
   {
      printf("To the moon!!\n");
   }
   impulse = Lambda[0] * w_vars->timeStep;
   return impulse;
}

size_t ContactConstraint::NumIter(void) const
{
   return 200;
}
