
#include "collision.h"
#include "gjk.h"
#include "sat.h"

void gjk_collide(std::vector<Body>& bodies, std::vector<Collision>& collisions)
{
   collisions.clear();
   for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); it++)
   {
      for (std::vector<Body>::iterator jt = it; jt != bodies.end(); jt++)
      {
         if (it != jt)
         {
            Collision coll;
            coll.body_one = &(*it);
            coll.body_two = &(*jt);
            if (gjk_check_collision(*(it->form), *(jt->form), &gjk_support, coll))
            {
               collisions.push_back(coll);
            }
         }
      }
   }
}

Collision sat_collide(Body* bodyA, Body* bodyB)
{
   Collision coll;
   coll.body_one = bodyA;
   coll.body_two = bodyA;

   /*shape* shapeA = bodyA->form;
   shape* shapeB = bodyB->form;
   if (shapeA->type == sh_rectangle && shapeB->type == sh_rectangle)
   {
      rectangle* rectA = static_cast<rectangle*>(shapeA);
      rectangle* rectB = static_cast<rectangle*>(shapeB);
      Vector2 p;
      Segment e;
      int shapeNum = check_collide(*rectA, *rectB, p, e);
      coll.one.p = p;
      if (shapeNum == 0)
      {
         coll.body_one = bodyA;
         coll.body_two = bodyB;
      }
      if (shapeNum == 1)
      {
         coll.body_one = bodyB;
         coll.body_two = bodyA;
      }
   }
   if (shapeA->type == sh_circle && shapeB->type == sh_circle)
   {
      circle* circA = static_cast<circle*>(shapeA);
      circle* circB = static_cast<circle*>(shapeB);
      Vector2 direction = circB->point - circA->point;
      direction.normalize2();
      coll.normal = direction;
      coll.one.p = circA->point + direction * circA->radius;
   }*/

   return coll;
}