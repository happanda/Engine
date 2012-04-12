#include "Constraint.h"

Constraint::Constraint(Body* bodyA, Body* bodyB):
bodyA(bodyA), bodyB(bodyB)
{
}

size_t Constraint::NumIter() const
{
   return Constraint::MAX_ITER;
}