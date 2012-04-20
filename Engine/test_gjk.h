#ifndef TEST_GJK_H
#define TEST_GJK_H

#include <vector>
#include "World\World.h"
#include "Body\Body.h"
#include "Graphics\Draw.h"
#include "glut/glut.h"

extern World world;
extern clock_t prevCl;
extern void main_init();

void test_gjk_mouse(int btn, int state, int x, int y) {}
void test_gjk_motion(int x, int y) {}
void test_gjk_passiveMotion(int x, int y) {}

void test_gjk_key(unsigned char key, int x, int y)
{
   double angle_delta = 0.2,
      move_delta = 0.2;
   int body_num = 0;
   double angle = 0,
      move_x = 0, move_y = 0;
   if (key == 'q')
      exit(0);
   if (key == 'a' || key == 's' || key == 'd' || key == 'z' || key == 'x' || key == 'c')
      body_num = 0;
   if (key == 'u' || key == 'i' || key == 'o' || key == 'j' || key == 'k' || key == 'l')
      body_num = 1;
   if (key == 'z' || key == 'j')
      move_x = -move_delta;
   if (key == 'c' || key == 'l')
      move_x = move_delta;
   if (key == 's' || key == 'i')
      move_y = move_delta;
   if (key == 'x' || key == 'k')
      move_y = -move_delta;
   if (key == 'a' || key == 'u')
      angle = angle_delta;
   if (key == 'd' || key == 'o')
      angle = -angle_delta;
   shape* sh = world.bodies[body_num].form;
   sh->alpha += angle;
   sh->point.v1 += move_x;
   sh->point.v2 += move_y;
}

void test_gjk_spec_key(int key, int x, int y)
{
}

void test_gjk_draw()
{
   clock_t cl = clock();
   if (cl > prevCl + (double)CLOCKS_PER_SEC * 20 / 1000)
   {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      draw_bodies(world.bodies);
      draw_collisions(world.collisions);
      world.update(world.vars.timeStep);
      draw_point(Vector2(0, 0));
      prevCl = cl;
      glutSwapBuffers();
   }
}

void test_gjk_init()
{
   world.init();
   world.vars.GRAVITATION = Vector2::ORIGIN;
   Body body = Body(&(rectangle()), 0, 0, 0, 0);
   rectangle* rect;

   rect = new rectangle(0, 0, 0, 8, 6);
   body = Body(rect, 4, 0, 0, 0);
   world.addBody(body);
   delete rect;

   rect = new rectangle(0, 5, 0, 5, 5);
   body = Body(rect, 4, 0, 0, 0);
   world.addBody(body);
   delete rect;

   glutKeyboardFunc(test_gjk_key);
   glutMouseFunc(test_gjk_mouse);
   glutMotionFunc(test_gjk_motion);
   glutPassiveMotionFunc(test_gjk_passiveMotion);
   glutSpecialFunc(test_gjk_spec_key);

   glutDisplayFunc(test_gjk_draw);
   glutIdleFunc(test_gjk_draw);
}

#endif