
#include "Body.h"

Body::Body(const shape* sh, double mazz, double velocity_x, double velocity_y,
           double angle_vel): mass(mazz), iMass(1 / mazz),
           velocity(Vector2(velocity_x, velocity_y)),
           angle_vel(angle_vel)
{
   if (sh->type == sh_rectangle)
   {
      const rectangle* rect = static_cast<const rectangle*>(sh);
      form = new rectangle(sh->point.v1, sh->point.v2, sh->alpha,
         rect->h, rect->w);
      inert = mass * (rect->h * rect->h + rect->w * rect->w) / 12;// inertia of rectangle
   }
   if (sh->type == sh_circle)
   {
      const circle* circ = static_cast<const circle*>(sh);
      form = new circle(sh->point.v1, sh->point.v2, sh->alpha,
         circ->radius);
      inert = mass * (3.14 * circ->radius * circ->radius
          * circ->radius * circ->radius) / 4;// inertia of circle
   }
   iInert = 1 / inert;
}

Body::Body(const Body& body): mass(body.mass), iMass(body.iMass),
velocity(body.velocity), angle_vel(body.angle_vel), inert(body.inert), iInert(body.iInert)
{
   if (body.form->type == sh_rectangle)
   {
      rectangle* rect = static_cast<rectangle*>(body.form);
      form = new rectangle(body.form->point.v1, body.form->point.v2, body.form->alpha,
         rect->h, rect->w);
   }
   if (body.form->type == sh_circle)
   {
      circle* circ = static_cast<circle*>(body.form);
      form = new circle(body.form->point.v1, body.form->point.v2, body.form->alpha,
         circ->radius);
      inert = mass * (3.14 * circ->radius * circ->radius
          * circ->radius * circ->radius) / 4;// inertia of circle
   }
}

const Body& Body::operator=(const Body& body)
{
   mass = body.mass;
   iMass = body.iMass;
   velocity = body.velocity;
   angle_vel = body.angle_vel;
   inert = body.inert;
   iInert = body.iInert;
   if (body.form->type == sh_rectangle)
   {
      rectangle* rect = static_cast<rectangle*>(body.form);
      form = new rectangle(body.form->point.v1, body.form->point.v2, body.form->alpha,
         rect->h, rect->w);
   }
   else if (body.form->type == sh_circle)
   {
      circle* circ = static_cast<circle*>(body.form);
      form = new circle(body.form->point.v1, body.form->point.v2, body.form->alpha,
         circ->radius);
   }
   return *this;
}

Vector2 Body::point_velocity(Vector2 point)
{
   Vector2 r = point - form->point;
   return velocity + Vector2(-r.v2 * angle_vel, r.v1 * angle_vel);
}

Body::~Body()
{
   if (form != 0)
   {
      if (form->type == sh_rectangle)
      {
         rectangle* rect = static_cast<rectangle*>(form);
         delete rect;
      }
      if (form->type == sh_circle)
      {
         circle* circ = static_cast<circle*>(form);
         delete circ;
      }
      if (form->type == sh_surface)
      {
         surface* surf = static_cast<surface*>(form);
         delete surf;
      }
   }
}