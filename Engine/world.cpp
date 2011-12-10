
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
   restitution = 0.7;
   friction = 0.75;
   timeStep = 20;
   gravitation = Vector2(0, 0);
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

            j = (-(1 + restitution) * rel_vel) * (m1 * m2) / (m1 + m2);
            Vector2 deltaV1 = it->normal * j;
            if (m1 < 5000)
               it->body_one->velocity = it->body_one->velocity + deltaV1 * (1 / m1)
               + tang * ((1 - friction) * -vel_one_tang);
            Vector2 deltaV2 = -deltaV1;
            if (m2 < 5000)
               it->body_two->velocity = it->body_two->velocity + deltaV2 * (1 / m2)
               + tang * ((1 - friction) * -vel_two_tang);

            Vector3 deltaV1_3d(deltaV1.v1, deltaV1.v2, 0);
            if (m1 < 5000)
               it->body_one->angle_vel += iinrt1 * ra.cross(deltaV1_3d).v3;
            Vector3 deltaV2_3d(deltaV2.v1, deltaV2.v2, 0);
            if (m2 < 5000)
               it->body_two->angle_vel += iinrt2 * ra.cross(deltaV2_3d).v3;
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