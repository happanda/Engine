
#include "world.h"
#include <stdio.h>
#include "gjk.h"
#include "math_routines.h"
#include "draw.h"

World::World()
{
   timeStep = 20;
}

void World::init()
{
   bodies.clear();
   collisions.clear();
   timeStep = 20;
}

void World::update(double deltaT)
{
   gjk_collide(bodies, collisions);
   resolve_collision();
   for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); it++)
   {
      (it)->form->rotate((it)->angle_vel * deltaT);
      (it)->form->point = (it)->form->point + (it)->velocity * deltaT;
   }
}

void World::resolve_collision()
{
   for (std::vector<Collision>::iterator it = collisions.begin(); it != collisions.end(); it++)
   {
      if (it->edge_edge)
      {
         Vector2 tang = it->normal.perpendicular();
         double vel_one_norm = it->body_one->velocity * it->normal;
         double vel_two_norm = it->body_two->velocity * it->normal;
         double rel_vel = vel_one_norm - vel_two_norm;
         if (rel_vel < 0)
         {
            double vel_one_tang = it->body_one->velocity * tang;
            double vel_two_tang = it->body_two->velocity * tang;
            double m1 = it->body_one->mass;
            double m2 = it->body_two->mass;
            double msum = m1 + m2;
            it->body_one->velocity = it->normal
               * ((m1 - m2)/msum * vel_one_norm + 2 * m2 / msum * vel_two_norm)
               + tang * vel_one_tang;
            it->body_two->velocity = (it->normal)
               * ((m2 - m1)/msum * vel_two_norm + 2 * m1 / msum * vel_one_norm)
               + tang * vel_two_tang;
         }
      }
      else
      {
         Vector2 tang = it->normal.perpendicular();
         double vel_one_norm = it->body_one->velocity * it->normal;
         double vel_two_norm = it->body_two->velocity * it->normal;
         double rel_vel = vel_one_norm - vel_two_norm;
         double m1 = it->body_one->mass;
         double m2 = it->body_two->mass;
         double iinrt1 = 1 / it->body_one->inert;
         double iinrt2 = 1 / it->body_two->inert;
         if (rel_vel < 0)
         {
            double j = 0;
            Vector3 n3(it->normal.v1, it->normal.v2, 0);

            Vector2 ra_2d = it->one.p - it->body_one->form->point;
            Vector3 ra(ra_2d.v1, ra_2d.v2, 0);
            Vector3 ra_copy = ra;
            Vector3 a_add3 = (ra.cross(n3)).cross(ra_copy);
            Vector2 a_add2(a_add3.v1, a_add3.v2);

            Vector2 rb_2d = it->two.p - it->body_two->form->point;
            Vector3 rb(rb_2d.v1, rb_2d.v2, 0);
            Vector3 rb_copy = ra;
            Vector3 b_add3 = (rb.cross(n3)).cross(rb_copy);
            Vector2 b_add2(b_add3.v1, b_add3.v2);

            double vel_one_tang = it->body_one->velocity * tang;
            double vel_two_tang = it->body_two->velocity * tang;

            j = (-2 * rel_vel) / (1/m1 + 1/m2);
            it->body_one->velocity = it->body_one->velocity + (it->normal * (j / m1));
            it->body_two->velocity = it->body_two->velocity - (it->normal * (j / m2));


            /*Vector3 Jn(j * it->normal.v1, j * it->normal.v2, 0);

            Vector2 pn_one = it->normal * (vel_one_norm * m1);
            Vector3 pn_one3(pn_one.v1, pn_one.v2, 0);
            Vector3 cross_one = pn_one3.cross(Jn);
            double ang_one_delta = cross_one.v3 * iinrt1;
            it->body_one->angle_vel += ang_one_delta;

            Vector2 pn_two = it->normal * (vel_two_norm * m2);
            Vector3 pn_two3(pn_two.v1, pn_two.v2, 0);
            Vector3 cross_two = pn_two3.cross(Jn);
            double ang_two_delta = cross_two.v3 * iinrt2;
            it->body_two->angle_vel += ang_two_delta;*/
         }
      }
   }
}
