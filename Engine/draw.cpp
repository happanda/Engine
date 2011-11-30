#include "draw.h"

GLfloat light_diffuse[] = { 1.0, 0.0, 0.0, 1.0 };
GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

void init_screen()
{
   //glEnable(GL_DEPTH_TEST);
   glMatrixMode(GL_PROJECTION);
   gluPerspective( /* field of view in degrees */ 100.0,
      /* aspect ratio */ 1.0,
      /* Z near */ 1.0, /* Z far */ 50.0);
   glMatrixMode(GL_MODELVIEW);
   gluLookAt(0.0, 0.0, 10.0, /* eye is at (0,0,5) */
      0.0, 0.0, 0.0, /* center is at (0,0,0) */
      0.0, 1.0, 0.0); /* up is in +Y direction */
   glTranslatef(0.0, 0.0, -1.0);
   glClearColor(1, 1, 1, 0);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void draw_bodies(const std::vector<Body>& bodies)
{
   glColor3f(0.0f, 0.0f, 0.0f);
   int numBodies = bodies.size();
   for (int i = 0; i < numBodies; i++)
   {
      if (bodies[i].form->type == sh_rectangle)
      {
         rectangle* rect = static_cast<rectangle*>(bodies[i].form);
         Vector2 p1, p2, p3, p4;
         get_points(*rect, p1, p2, p3, p4, 0);

         glBegin(GL_LINE_LOOP);
         glVertex2f(p1.v1, p1.v2);
         glVertex2f(p2.v1, p2.v2);
         glVertex2f(p3.v1, p3.v2);
         glVertex2f(p4.v1, p4.v2);
         glEnd();
      }
      if (bodies[i].form->type == sh_circle)
      {
         circle* circ = static_cast<circle*>(bodies[i].form);
         GLfloat twoPi = 2.0f * 3.14159f;

         glBegin(GL_LINE_LOOP);
         glVertex2f(bodies[i].form->point.v1, bodies[i].form->point.v2);
         for (int numsec = 0; numsec <= CIRCLE_SEGMENTS; numsec++)
         {
            glVertex2f(bodies[i].form->point.v1 + circ->radius
               * cos(bodies[i].form->alpha + numsec *  twoPi / CIRCLE_SEGMENTS),
               bodies[i].form->point.v2 + circ->radius
               * sin(bodies[i].form->alpha + numsec * twoPi / CIRCLE_SEGMENTS));
         }
         glEnd();
      }
      if (bodies[i].form->type == sh_surface)
      {
         double faraway = SURFACE_DISTANCE;
         surface* surf = static_cast<surface*>(bodies[i].form);

         glBegin(GL_LINE_LOOP);
         glVertex2f(surf->point.v1 + faraway * cos(surf->alpha),
            surf->point.v2 + faraway * sin(surf->alpha));
         glVertex2f(surf->point.v1 - faraway * cos(surf->alpha),
            surf->point.v2 - faraway * sin(surf->alpha));
         glEnd();
      }
   }
}

void draw_collisions(const std::vector<Collision>& collisions)
{
   glColor3f(1.0f, 0.0f, 0.0f);
   for (std::vector<Collision>::const_iterator it = collisions.begin();
      it != collisions.end(); it++)
   {
      if (!it->edge_edge)
      {
         draw_point(it->one.p);
         draw_point(it->two.p);
         glColor3f(0.0f, 0.0f, 1.0f);
         draw_segment(Segment(it->two.p, it->two.p + it->normal));
      }
      else
      {
         draw_point(it->one.segm.head);
         draw_point(it->one.segm.tail);
         draw_point(it->two.segm.head);
         draw_point(it->two.segm.tail);
         glColor3f(0.0f, 0.0f, 1.0f);
         Vector2 from = (it->two.segm.head + it->two.segm.tail) * 0.5;
         draw_segment(Segment(from, from + it->normal));
      }
   }
}

void draw_point(Vector2 point)
{
   double small_radius = 0.1;
   int segments = 20;
   GLfloat twoPi = 2.0f * 3.14159f;
   glBegin(GL_POINTS);
   for (int numsec = 0; numsec <= segments; numsec++)
   {
      glVertex2f(point.v1 + small_radius * cos(numsec *  twoPi / segments),
         point.v2 + small_radius * sin(numsec * twoPi / segments));
   }
   glEnd();
}

void draw_segment(Segment segment)
{
   glBegin(GL_LINE_LOOP);
   glVertex2f(segment.head.v1, segment.head.v2);
   glVertex2f(segment.tail.v1, segment.tail.v2);
   glEnd();
}

void draw_simplex(Simplex simplex)
{
   glBegin(GL_LINE_LOOP);
   for (std::deque<Vector2>::iterator it = simplex.P.begin(); it != simplex.P.end(); it++)
      glVertex2f(it->v1, it->v2);
   glEnd();
}