#include "CollisionContraint.h"
#include "body.h"
#include "Vector2.h"
#include "Vector3.h"
#include "world.h"
#include "PGSsolver.h"
#include "float.h"

CollisionContraint::CollisionContraint(Collision* collision):
Constraint(collision->body_one, collision->body_two), _collision(collision)
{
   _sum_impulse_n = 0;
   _sum_impulse_t = 0;
   Body* bA = _collision->body_one;
   Body* bB = _collision->body_two;
   Jacobian = std::vector<std::vector<double>>(1);
   Jacobian[0] = std::vector<double>(6);
   Vector2 norm = _collision->normal;
   Vector3 norm3(norm.v1, norm.v2, 0);
   Vector3 r3(_collision->one[0].v1, _collision->one[0].v2, 0);
   r3.cross(norm3);
   Jacobian[0][0] = -norm.v1;
   Jacobian[0][1] = -norm.v2;
   Jacobian[0][2] = -r3.v3;
   r3 = Vector3(_collision->two[0].v1, _collision->two[0].v2, 0);
   r3.cross(norm3);
   Jacobian[0][3] = norm.v1;
   Jacobian[0][4] = norm.v2;
   Jacobian[0][5] = r3.v3;
   std::vector<double> mass(6);
   mass[0] = 1 / bA->mass;
   mass[1] = mass[0];
   mass[2] = bA->inert;
   mass[3] = 1 / bB->mass;
   mass[4] = mass[3];
   mass[5] = bB->inert;
   A = std::vector<std::vector<double>>(1);
   A[0] = std::vector<double>(1);
   A[0][0] = 0;
   for (size_t i = 0; i < 6; i++)
      A[0][0] += Jacobian[0][i] * mass[i];

   Eta = std::vector<double>(1);
   Eta[0] = bA->velocity.v1 * Jacobian[0][0]
   + bA->velocity.v2 * Jacobian[0][1]
   + bA->angle_vel * Jacobian[0][2]
   + bB->velocity.v1 * Jacobian[0][3]
   + bB->velocity.v2 * Jacobian[0][4]
   + bB->angle_vel * Jacobian[0][5];
   Eta[0] /= 20;//World::vars.timeStep;;
   Lambda = std::vector<double>(1, 1);
}

Vector2 CollisionContraint::ImpulseDirection() const
{
   return _collision->normal;
}

void CollisionContraint::Init(Vector2 ForceExternal)
{
}

double CollisionContraint::DeltaImpulse() const
{
   std::vector<double> L(1);
   L[0] = Lambda[0];
   SolveLambda(A, Eta, L, 0, DBL_MAX);
   //Lambda[0] = Lambda[0] + L[0];
   return L[0] * 20;//World::vars.timeStep;
}