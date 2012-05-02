#include "Constraint.h"

Constraint::Constraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars):
bodyA(bodyA), bodyB(bodyB), rA(rA), rB(rB), w_vars(vars)
{
}

size_t Constraint::NumIter(void) const
{
   return Constraint::MAX_ITER;
}

void Constraint::DeltaImpulse()
{
   impulse = _deltaImpulse();
   impulseDirection = _impulseDirection();
}

void Constraint::ApplyImpulse()
{
   Vector2 imp = impulseDirection * impulse;
   if (bodyA != NULL && bodyA->mass < w_vars->UNMOVABLE_MASS)
   {
      bodyA->velocity = bodyA->velocity + imp * bodyA->iMass;
      double iinrt1 = bodyA->iInert;
      Vector3 imp3(imp.v1, imp.v2, 0);
      Vector3 r3(rA.v1, rA.v2, 0);
      r3 = r3.cross(imp3);
      double ro1 = r3.v3;
      bodyA->angle_vel += iinrt1 * ro1;
   }
   if (bodyB != NULL && bodyB->mass < w_vars->UNMOVABLE_MASS)
   {
      bodyB->velocity = bodyB->velocity - imp * bodyB->iMass;
      double iinrt2 = bodyB->iInert;
      Vector3 imp3(imp.v1, imp.v2, 0);
      Vector3 r3(rB.v1, rB.v2, 0);
      r3 = r3.cross(imp3);
      double ro2 = r3.v3;
      bodyB->angle_vel -= iinrt2 * ro2;
   }
}