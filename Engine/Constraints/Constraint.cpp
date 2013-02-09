#include <iostream>
#include "Constraint.h"

Constraint::Constraint(Body* bodyA, Vector2 rA, Body* bodyB, Vector2 rB, world_vars* vars):
    bodyA(bodyA), bodyB(bodyB), rA(rA), rB(rB), w_vars(vars)
{
    Type = BASE_CONSTRAINT;
    iterCount = 0;
    sum_impulse = 0;
    Impulse = Vector2::ORIGIN;
    Torque = 0;
    for (size_t i = 0; i < 6; i++) ForceExt[i] = 0;
}

bool Constraint::Enough() const
{
    return iterCount >= NumIter();
}
size_t Constraint::NumIter(void) const
{
   return Constraint::MAX_ITER;
}

void Constraint::SetForceExt(double* force)
{
    for (size_t i = 0; i < 6; i++) ForceExt[i] = force[i];
}

void Constraint::DeltaImpulse()
{
   _deltaImpulse(Impulse, Torque);
}

void Constraint::ApplyImpulse()
{
   if (bodyA != NULL && bodyA->mass < w_vars->UNMOVABLE_MASS)
   {
      bodyA->velocity = bodyA->velocity + Impulse * bodyA->iMass;
      double iinrt1 = bodyA->iInert;
      Vector3 imp3(Impulse.v1, Impulse.v2, 0);
      Vector3 r3(rA.v1, rA.v2, 0);
      r3 = r3.cross(imp3);
      double ro1 = r3.v3;

      bodyA->angle_vel += iinrt1 * ro1;
      bodyA->angle_vel += iinrt1 * Torque;
   }
   if (bodyB != NULL && bodyB->mass < w_vars->UNMOVABLE_MASS)
   {
      bodyB->velocity = bodyB->velocity - Impulse * bodyB->iMass;
      double iinrt2 = bodyB->iInert;
      Vector3 imp3(Impulse.v1, Impulse.v2, 0);
      Vector3 r3(rB.v1, rB.v2, 0);
      r3 = r3.cross(imp3);
      double ro2 = r3.v3;
      bodyB->angle_vel -= iinrt2 * ro2;
      bodyB->angle_vel -= iinrt2 * Torque;
   }
}

void Constraint::Fix()
{
}

Constraint::Constraint(const Constraint& constr):
    Type(constr.Type), bodyA(constr.bodyA), bodyB(constr.bodyB),
    rA(constr.rA), rB(constr.rB), w_vars(constr.w_vars)
{
    iterCount = constr.iterCount;
    sum_impulse = 0;
    Impulse = constr.Impulse;
    Torque = constr.Torque;
    for (size_t i = 0; i < 6; i++) ForceExt[i] = constr.ForceExt[i];
}