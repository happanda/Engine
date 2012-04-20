
#include "world.h"
#include <stdio.h>
#include "Collision\GJK.h"
#include "Math\MathRoutines.h"
#include "Graphics\Draw.h"
#include "Constraints\ContactConstraint.h"
#include "Constraints\FrictionConstraint.h"

World::World()
{
   init();
}

void World::init()
{
   bodies.clear();
   collisions.clear();
   vars.timeStep = 0.016;
   vars.iTimeStep = 1 / vars.timeStep;
   vars.UNMOVABLE_MASS = 5000;
   vars.RESTITUTION = 0.5;
   vars.FRICTION = 0.4;
   vars.GRAVITATION = Vector2(0, -9.8);
}

void World::update(double deltaT)
{
   gjk_collide(bodies, collisions);
   resolve_collision(deltaT);
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

void World::resolve_collision(double deltaT)
{
   const double min_impulse = 0.0000001;
   const double bias_factor = 0.3;
   const double delta_slop = 0.002;
   for (std::vector<Collision>::iterator it = collisions.begin(); it != collisions.end(); it++)
   {
      if (it->body_one->mass < vars.UNMOVABLE_MASS
         || it->body_two->mass < vars.UNMOVABLE_MASS)
      {
         ContactConstraint cc(&(*it), &vars);
         for (size_t num_iter = 0; num_iter < cc.NumIter() && !cc.Enough(); num_iter++)
         {
            cc.Init(Vector2::ORIGIN);
            cc.DeltaImpulse();
            cc.ApplyImpulse();
            if (cc.impulse < min_impulse)
               break;
         }
         /*FrictionConstraint fc(&(*it), &vars);
         for (size_t num_iter = 0; num_iter < fc.NumIter() && !fc.Enough(); num_iter++)
         {
            fc.Init(Vector2::ORIGIN);
            fc.DeltaImpulse();
            fc.ApplyImpulse();
            if (fc.impulse < min_impulse)
               break;
         }*/
      }
   }
}

void World::resolve_collision_old(double deltaT)
{
   const double min_impulse = 0.0000001;
   const double bias_factor = 0.3;
   const double delta_slop = 0.002;
   for (std::vector<Collision>::iterator it = collisions.begin(); it != collisions.end(); it++)
   {
      Vector2 tang = it->normal.perpendicular();

      int maxIter = 100;
      int numIter = 0;
      bool enough = false;
      double sum_impulse_n = 0,// normal direction total impulse
         sum_impulse_t = 0;// tangential direction total impulse

      // iteratively apply impulses
      while (!enough && numIter < maxIter)
      {
         numIter++;
         // iterate through collision points
         for (size_t pind = 0; pind < it->one.size(); pind++)
         {
            Vector2 vel_one = it->body_one->point_velocity(it->one.at(pind));
            Vector2 vel_two = it->body_two->point_velocity(it->two.at(pind));
            Vector2 vel_rel = vel_one - vel_two;
            double vel_rel_n = vel_rel * it->normal;
            if (vel_rel_n < 0)
            {
               double vel_rel_t = vel_rel * tang;

               double m1 = it->body_one->mass;
               double m2 = it->body_two->mass;
               double iinrt1 = it->body_one->iInert;
               double iinrt2 = it->body_two->iInert;

               // relative positions of contact points
               Vector2 ra_2d = it->one.at(pind) - it->body_one->form->point;
               Vector2 rb_2d = it->two.at(pind) - it->body_two->form->point;
               Vector2 a_add_n(0, 0), b_add_n(0, 0),
                  a_add_t(0, 0), b_add_t(0, 0);
               double mm = 0;

               if (m1 < vars.UNMOVABLE_MASS)
               {
                  mm += it->body_one->iMass;
                  a_add_n = cross_cross(ra_2d, it->normal) * iinrt1;
                  a_add_t = cross_cross(ra_2d, tang) * iinrt1;
               }
               if (m2 < vars.UNMOVABLE_MASS)
               {
                  mm += it->body_two->iMass;
                  b_add_n = cross_cross(rb_2d, it->normal) * iinrt2;
                  b_add_t = cross_cross(rb_2d, tang) * iinrt2;
               }
               
               double kn = mm + it->normal * (a_add_n + b_add_n);
               double kt = mm + tang * (a_add_t + b_add_t);
               double delta = (it->one.at(pind) - it->two.at(pind)).norm2();
               double v_bias = 0;
               // penetration correction impulse
               if (delta > delta_slop)
                  v_bias = bias_factor * (delta - delta_slop) / deltaT;
               double Pn = (-(1 + vars.RESTITUTION) * vel_rel_n + v_bias) / kn;
               double Pt = -vars.FRICTION * vel_rel_t / kt;

               if (Pn > 0 && Pt < -vars.FRICTION * Pn)
                  Pt = -vars.FRICTION * Pn;
               else if (Pn > 0 && Pt > vars.FRICTION * Pn)
                  Pt = vars.FRICTION * Pn;

               // total impulse
               Vector2 P = it->normal * Pn + tang * Pt;

               if (abs(Pn) < min_impulse && abs(Pt) < min_impulse)
                  enough = true;
               // correcting impulses
               if (sum_impulse_n + Pn < 0)
                  Pn = -sum_impulse_n;
               sum_impulse_n += Pn;
               if (sum_impulse_t + Pt < 0)
                  Pt = -sum_impulse_t;
               sum_impulse_t += Pt;

               Vector2 deltaV1(0, 0);
               Vector2 deltaV2(0, 0);

               double deltaW1 = 0;
               double deltaW2 = 0;

               if (m1 < vars.UNMOVABLE_MASS)
               {
                  deltaV1 = P * it->body_one->iMass;
                  deltaW1 = iinrt1 * (ra_2d.v1 * P.v2 - ra_2d.v2 * P.v1);
               }
               if (m2 < vars.UNMOVABLE_MASS)
               {
                  deltaV2 = P * (-it->body_two->iMass);
                  deltaW2 = -iinrt2 * (rb_2d.v1 * P.v2 - rb_2d.v2 * P.v1);
               }
               it->body_one->velocity = it->body_one->velocity + deltaV1;
               it->body_one->angle_vel += deltaW1;
               it->body_two->velocity = it->body_two->velocity + deltaV2;
               it->body_two->angle_vel += deltaW2;
            }
            else
               enough = true;
         }// for points
      }// while iterations
   }// for collisions
}

void World::apply_forces(double deltaT)
{
   for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); it++)
   {
      if (it->mass < vars.UNMOVABLE_MASS)
         it->velocity = it->velocity + vars.GRAVITATION * deltaT;
   }
}