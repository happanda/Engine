#include "Constraint.h"

Constraint::Constraint(Body* bodyA, Body* bodyB, world_vars* vars):
bodyA(bodyA), bodyB(bodyB), w_vars(vars)
{
}

size_t Constraint::NumIter(void) const
{
   return Constraint::MAX_ITER;
}