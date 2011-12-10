
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
            double vel_rel_t = vel_rel * tang;

            double m1 = it->body_one->mass;
            double m2 = it->body_two->mass;
            double iinrt1 = 1 / it->body_one->inert;
            double iinrt2 = 1 / it->body_two->inert;

            Vector2 ra_2d = it->one.p - it->body_one->form->point;
            Vector2 rb_2d = it->two.p - it->body_two->form->point;
            Vector2 a_add_n = cross_cross(ra_2d, it->normal) * iinrt1;
            Vector2 b_add_n = cross_cross(rb_2d, it->normal) * iinrt2;
            Vector2 a_add_t = cross_cross(ra_2d, tang) * iinrt1;
            Vector2 b_add_t = cross_cross(rb_2d, tang) * iinrt2;

            double mm = (m1 + m2) / (m1 * m2);
            double kn = mm + it->normal * (a_add_n + b_add_n);
            double kt = mm + tang * (a_add_t + b_add_t);
            double Pn = -(1 + restitution) * vel_rel_n / kn;
            double Pt = -(1 + restitution) * vel_rel_t / kt;

            if (Pn < 0) Pn = 0;
            if (Pt < -friction * Pn) Pt = -friction * Pn;
            else if (Pt > friction * Pn) Pt = friction * Pn;

            Vector2 P = it->normal * Pn + tang * Pt;
            if (Pn < 0.01 && Pt < 0.01)
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

void World::apply_forces(double deltaT)
{
   for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); it++)
   {
      if (it->mass < 5000)
         it->velocity = it->velocity + gravitation * deltaT;
   }
}