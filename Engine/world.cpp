
#include "world.h"
#include <stdio.h>
#include "gjk.h"
#include "math_routines.h"
#include "draw.h"

World::World()
{
   init();
}

void World::init()
{
   bodies.clear();
   collisions.clear();
   restitution = 0.2;
   friction = 0.75;
   timeStep = 20;
   gravitation = Vector2(0, -9.8);
}

void World::update(double deltaT)
{
   gjk_collide(bodies, collisions);
   resolve_collision();
   apply_forces(deltaT);
   double energy = 0;
   for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); it++)
   {
      it->form->rotate(it->angle_vel * deltaT);
      it->form->point = it->form->point + it->velocity * deltaT;
      energy += (it->mass * it->velocity.norm2sq()
         + it->inert * it->angle_vel * it->angle_vel) / 2;
   }
   printf("Total kinetic energy: %.2f\n", energy);
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
               + tang * (friction * vel_one_tang);
            it->body_two->velocity = (it->normal)
               * ((m2 - m1)/msum * vel_two_norm + 2 * m1 / msum * vel_one_norm)
               + tang * (friction * vel_two_tang);
         }
      }
      else
      {
         Vector2 tang = it->normal.perpendicular();

         int maxIter = 100;
         int numIter = 0;
         bool enough = false;
         double summary_impulse = 0;
         while (!enough && numIter < maxIter)
         {
            numIter++;
            Vector2 vel_one = it->body_one->point_velocity(it->one.p);
            Vector2 vel_two = it->body_two->point_velocity(it->two.p);
            Vector2 vel_rel = vel_one - vel_two;
            double vel_rel_n = vel_rel * it->normal;
            if (vel_rel_n < 0)
            {
               double m1 = it->body_one->mass;
               double m2 = it->body_two->mass;
               double iinrt1 = 1 / it->body_one->inert;
               double iinrt2 = 1 / it->body_two->inert;

               Vector3 n3(it->normal.v1, it->normal.v2, 0);

               Vector2 ra_2d = it->one.p - it->body_one->form->point;
               Vector3 ra(ra_2d.v1, ra_2d.v2, 0);
               Vector3 a_add3 = (ra.cross(n3)).cross(ra);
               Vector2 a_add2 = Vector2(a_add3.v1, a_add3.v2) * iinrt1;

               Vector2 rb_2d = it->two.p - it->body_two->form->point;
               Vector3 rb(rb_2d.v1, rb_2d.v2, 0);
               Vector3 b_add3 = (rb.cross(n3)).cross(rb);
               Vector2 b_add2 = Vector2(b_add3.v1, b_add3.v2) * iinrt2;

               double vel_one_tang = vel_one * tang;
               double vel_two_tang = vel_two * tang;

               double kn = ((m1 + m2) / (m1 * m2) + it->normal * (a_add2 + b_add2));
               double Pn = -(1 + restitution) * vel_rel_n / kn;
               if (Pn < 0)
                  Pn = 0;
               Vector2 P = it->normal * Pn;
               if (Pn < 0.01)
                  enough = true;
               if (summary_impulse + Pn < 0)
                  Pn = -summary_impulse;
               summary_impulse += Pn;

               Vector2 deltaV1 = P * (1 / m1);
               Vector2 deltaV2 = P * (-1 / m2);
               double deltaW1 = iinrt1 * (ra_2d.v1 * P.v2 - ra_2d.v2 * P.v1);
               double deltaW2 = -iinrt2 * (rb_2d.v1 * P.v2 - rb_2d.v2 * P.v1);

               if (m1 < 5000)
                  it->body_one->velocity = it->body_one->velocity + deltaV1;
               if (m2 < 5000)
                  it->body_two->velocity = it->body_two->velocity + deltaV2;

               if (m1 < 5000)
                  it->body_one->angle_vel += deltaW1;
               if (m2 < 5000)
                  it->body_two->angle_vel += deltaW2;
            }
            else
               enough = true;
         }
      }
   }
}

void World::apply_forces(double deltaT)
{
   for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); it++)
   {
      if (it->mass < 5000)
         it->velocity = it->velocity + gravitation * deltaT;
   }
}