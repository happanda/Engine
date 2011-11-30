#ifndef INCLUDE_BODY
#define INCLUDE_BODY

#include "shape.h"

class Body
{
public:
   shape* form;
   double mass;
   double inert;// inertia tensor

   Vector2 velocity;
   double angle_vel;

   Body(const shape* sh, double mazz, double velocity_x, double velocity_y,
      double angle_vel): mass(mazz), velocity(Vector2(velocity_x, velocity_y)),
      angle_vel(angle_vel)
   {
      if (sh->type == sh_rectangle)
      {
         const rectangle* rect = static_cast<const rectangle*>(sh);
         form = new rectangle(sh->point.v1, sh->point.v2, sh->alpha,
            rect->h, rect->w);
         inert = mass * (rect->h * rect->h + rect->w * rect->w) / 12;// inertia of rectangle
      }
   }

   Body(const Body& body): mass(body.mass),
      velocity(body.velocity), angle_vel(body.angle_vel), inert(body.inert)
   {
      if (body.form->type == sh_rectangle)
      {
         rectangle* rect = static_cast<rectangle*>(body.form);
         form = new rectangle(body.form->point.v1, body.form->point.v2, body.form->alpha,
            rect->h, rect->w);
      }
   }

   const Body& operator=(const Body& body)
   {
      mass = body.mass;
      velocity = body.velocity;
      angle_vel = body.angle_vel;
      inert = body.inert;
      if (body.form->type == sh_rectangle)
      {
         rectangle* rect = static_cast<rectangle*>(body.form);
         form = new rectangle(body.form->point.v1, body.form->point.v2, body.form->alpha,
            rect->h, rect->w);
      }
      return *this;
   }

   ~Body()
   {
      if (form != NULL)
      {
         if (form->type == sh_rectangle)
         {
            rectangle* rect = static_cast<rectangle*>(form);
            delete rect;
         }
      }
   }
};

#endif