#ifndef INCLUDE_COLLISION
#define INCLUDE_COLLISION

#include <vector>
#include "Math\MathRoutines.h"
#include "Body\Body.h"
#include "Body\Shape.h"

class Collision
{
public:
   Body* body_one;
   Body* body_two;
   std::vector<Vector2> one;
   std::vector<Vector2> two;
   Vector2 normal;
};

void gjk_collide(std::vector<Body>& bodies, std::vector<Collision>& collisions);

bool bbox_check_collision(Body* bodyA, Body* bodyB);

Collision sat_collide(Body* bodyA, Body* bodyB);

#endif