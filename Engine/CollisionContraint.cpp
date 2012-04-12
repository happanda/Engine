#include "CollisionContraint.h"

CollisionContraint::CollisionContraint(Collision* collision):
Constraint(collision->body_one, collision->body_two), _collision(collision)
{
   _sum_impulse_n = 0;
   _sum_impulse_t = 0;
}

Vector2 CollisionContraint::ImpulseDirection() const
{
   return _collision->normal;
}

void CollisionContraint::Init(Vector2 ForceExternal)
{
}

double CollisionContraint::DeltaImpulse() const
{
   //for (size_t pind = 0; pind < it->one.size(); pind++)
   //{
   //   Vector2 vel_one = it->body_one->point_velocity(it->one.at(pind));
   //   Vector2 vel_two = it->body_two->point_velocity(it->two.at(pind));
   //   Vector2 vel_rel = vel_one - vel_two;
   //   double vel_rel_n = vel_rel * it->normal;
   //   if (vel_rel_n < 0)
   //   {
   //      double vel_rel_t = vel_rel * tang;

   //      double m1 = it->body_one->mass;
   //      double m2 = it->body_two->mass;
   //      double iinrt1 = 1 / it->body_one->inert;
   //      double iinrt2 = 1 / it->body_two->inert;

   //      // relative positions of contact points
   //      Vector2 ra_2d = it->one.at(pind) - it->body_one->form->point;
   //      Vector2 rb_2d = it->two.at(pind) - it->body_two->form->point;
   //      Vector2 a_add_n(0, 0), b_add_n(0, 0),
   //         a_add_t(0, 0), b_add_t(0, 0);
   //      double mm = 0;

   //      if (m1 < vars.UNMOVABLE_MASS)
   //      {
   //         mm += 1 / m1;
   //         a_add_n = cross_cross(ra_2d, it->normal) * iinrt1;
   //         a_add_t = cross_cross(ra_2d, tang) * iinrt1;
   //      }
   //      if (m2 < vars.UNMOVABLE_MASS)
   //      {
   //         mm += 1 / m2;
   //         b_add_n = cross_cross(rb_2d, it->normal) * iinrt2;
   //         b_add_t = cross_cross(rb_2d, tang) * iinrt2;
   //      }
   //      
   //      double kn = mm + it->normal * (a_add_n + b_add_n);
   //      double kt = mm + tang * (a_add_t + b_add_t);
   //      double delta = (it->one.at(pind) - it->two.at(pind)).norm2();
   //      double v_bias = 0;
   //      // penetration correction impulse
   //      if (delta > delta_slop)
   //         v_bias = bias_factor * (delta - delta_slop) / deltaT;
   //      double Pn = (-(1 + vars.RESTITUTION) * vel_rel_n + v_bias) / kn;
   //      double Pt = -vars.FRICTION * vel_rel_t / kt;

   //      if (Pn > 0 && Pt < -vars.FRICTION * Pn)
   //         Pt = -vars.FRICTION * Pn;
   //      else if (Pn > 0 && Pt > vars.FRICTION * Pn)
   //         Pt = vars.FRICTION * Pn;

   //      // total impulse
   //      Vector2 P = it->normal * Pn + tang * Pt;

   //      if (abs(Pn) < min_impulse && abs(Pt) < min_impulse)
   //         enough = true;
   //      // correcting impulses
   //      if (sum_impulse_n + Pn < 0)
   //         Pn = -sum_impulse_n;
   //      sum_impulse_n += Pn;
   //      if (sum_impulse_t + Pt < 0)
   //         Pt = -sum_impulse_t;
   //      sum_impulse_t += Pt;

   //      Vector2 deltaV1(0, 0);
   //      Vector2 deltaV2(0, 0);

   //      double deltaW1 = 0;
   //      double deltaW2 = 0;

   //      if (m1 < vars.UNMOVABLE_MASS)
   //      {
   //         deltaV1 = P * (1 / m1);
   //         deltaW1 = iinrt1 * (ra_2d.v1 * P.v2 - ra_2d.v2 * P.v1);
   //      }
   //      if (m2 < vars.UNMOVABLE_MASS)
   //      {
   //         deltaV2 = P * (-1 / m2);
   //         deltaW2 = -iinrt2 * (rb_2d.v1 * P.v2 - rb_2d.v2 * P.v1);
   //      }
   //      it->body_one->velocity = it->body_one->velocity + deltaV1;
   //      it->body_one->angle_vel += deltaW1;
   //      it->body_two->velocity = it->body_two->velocity + deltaV2;
   //      it->body_two->angle_vel += deltaW2;
   //   }
   //   else
   //      enough = true;
   //}// for points
   return 0;
}