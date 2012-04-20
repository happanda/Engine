#include <stdio.h>
#include <time.h>
#include <vector>
#include <cstdlib>
#include <float.h>
#include "World\World.h"
#include "Body\Shape.h"
#include "Body\Body.h"
#include "Graphics\Draw.h"
#include "test_GJK.h"
#include "glut/glut.h"
#include "anttweakbar\AntTweakBar.h"


World world = World();
int speed = 20;
bool pause = false;
bool draw_colls = false;
bool draw_tw = true;

void init_bodies();
void glut_init();
void tw_init();
void stack_init();

clock_t prevCl = 0;
void step()
{
   clock_t cl = clock();
   if (cl > prevCl + (double)CLOCKS_PER_SEC * speed / 1000 && !pause)
   {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      draw_bodies(world.bodies);
      if (draw_colls)
         draw_collisions(world.collisions);
      world.update(world.vars.timeStep);
      prevCl = cl;
      if (draw_tw)
         TwDraw();
      glutSwapBuffers();
   }
}

void keyboard(unsigned char key, int x, int y)
{
   if (key == 'p')
      pause = !pause;
   if (key == 'q')
      exit(0);
   if (key == '\'')
      draw_colls = !draw_colls;
   if (key == 'y')
   {
      int r = rand() % world.bodies.size();
      if (world.bodies.at(r).mass < world.vars.UNMOVABLE_MASS)
         world.bodies.at(r).velocity = world.bodies.at(r).velocity + Vector2(0, 12);
   }
   if (draw_tw)
      TwEventKeyboardGLUT(key, x, y);
   glutBitmapCharacter(GLUT_BITMAP_8_BY_13, key);
   glutPostRedisplay();
}

void mouse(int btn, int state, int x, int y)
{
   TwEventMouseButtonGLUT(btn, state, x, y);
}

void motion(int x, int y)
{
   TwEventMouseMotionGLUT(x, y);
}

void passiveMotion(int x, int y)
{
   TwEventMouseMotionGLUT(x, y);
}

void choice_selected(int value)
{
   if (value == 1) { glut_init(); init_bodies(); tw_init(); draw_tw = true; }
   if (value == 2) { test_gjk_init(); draw_tw = false; }
   if (value == 3) { glut_init(); stack_init(); tw_init(); draw_tw = true; }
}

void specialKey(int key, int x, int y)
{
   if (key == GLUT_KEY_UP)
   {
      world.vars.timeStep += 0.005;
   }
   if (key == GLUT_KEY_DOWN)
   {
      world.vars.timeStep -= 0.005;
   }
   if (abs(world.vars.timeStep) > DBL_EPSILON)
         world.vars.iTimeStep = 1 / world.vars.timeStep;
}

void reshape(int width, int height)
{
   reshape_window(width, height);
   if (draw_tw)
      TwWindowSize(width, height);
}

int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitWindowSize(800, 600);
   glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
   glutCreateWindow("Engine");

   glut_init();
   tw_init();
   init_color();

   init_bodies();
   pause = false;

   glutMainLoop();
}

void glut_init()
{
   glutDisplayFunc(step);
   glutIdleFunc(step);
   glutKeyboardFunc(keyboard);
   glutMouseFunc(mouse);
   glutMotionFunc(motion);
   glutPassiveMotionFunc(passiveMotion);
   glutSpecialFunc(specialKey);
   glutReshapeFunc(reshape);

   glutCreateMenu(choice_selected);
   glutAddMenuEntry("Main simulation", 1);
   glutAddMenuEntry("Test GJK", 2);
   glutAddMenuEntry("Stack boxes", 3);
   glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void tw_init()
{
   // Initialize AntTweakBar
   TwInit(TW_OPENGL, NULL);

   // Create a tweak bar
   TwBar* bar = TwNewBar("TweakBar");
   TwDefine(" TweakBar size='200 100' color='100 100 100' ");
   TwAddVarRW(bar, "Restitution", TW_TYPE_DOUBLE, &world.vars.RESTITUTION,
      " min=0.0 max=1.0 step=0.05 help='Coefficient of restitution, default 0.5.' ");
   TwAddVarRW(bar, "Friction", TW_TYPE_DOUBLE, &world.vars.FRICTION,
      " min=0.0 max=1.0 step=0.05 help='Coefficient of restitution, default 0.4.' ");
   TwAddVarRW(bar, "Gravity", TW_TYPE_DOUBLE, &world.vars.GRAVITATION.v2,
      " min=-100 max=100 step=0.5 help='Coefficient of restitution, default -9.8.' ");
}

void init_bodies()
{
   world_vars wvars = world.vars;
   wvars.GRAVITATION.v2 = 0;
   wvars.RESTITUTION = 1;
   world.init();
   world.vars = wvars;

   double angle_vel = 0;
   std::vector<Body> bodies;

   Body body = Body(&(rectangle()), 0, 0, 0, 0);
   rectangle* rect;
   circle* circ;
   //surface* surf;
   //srand(time(NULL));
   /*int s = 12;
   double sp = (double)RAND_MAX / 3;
   for (int i = -s; i < s; i += 4)
   {
      for (int j = -s + 2; j < s; j += 4)
      {
         double r = rand() % 2 + 1;
         rect = new rectangle(i, j, (double)rand() / sp, r, 3 - r);
         body = Body(rect, 1, (double)rand() / sp - (double)rand() / sp,
            (double)rand() / sp - (double)rand() / sp, 0);
         bodies.push_back(body);
         delete rect;
      }
   }*/
   rect = new rectangle(0, 0, 2, 4, 4);
   body = Body(rect, 16, 0, 0, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(0, 5, -1, 2, 2);
   body = Body(rect, 4, 0, -2, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(-5, 0, 0, 2, 2);
   body = Body(rect, 4, 1, 0, angle_vel);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(0, -5, 0, 2, 2);
   body = Body(rect, 4, 0, 1, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(5, 0, 0, 2, 2);
   body = Body(rect, 4, -1, 0, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(10, 5, 0, 3, 1);
   body = Body(rect, 3, -3, -3, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(-9, -6, 0.2, 1, 10);
   body = Body(rect, 10, 2, 0, 0);
   bodies.push_back(body);
   delete rect;

   circ = new circle(-3, 4, 0, 2);
   body = Body(circ, 2, 0, -4, 0);
   bodies.push_back(body);
   delete circ;

   circ = new circle(4, 7, 0, 3);
   body = Body(circ, 8, -2, -2, 0);
   bodies.push_back(body);
   delete circ;

   //bodies.erase((bodies.begin() + 1), bodies.end());
   // bounds
   double bigmass = 100000;
   rect = new rectangle(-40, 0, 0, 50, 25);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   rect = new rectangle(40, 0, 0, 50, 25);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   rect = new rectangle(0, 14, 0, 200, 3);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   rect = new rectangle(0, -14, 0, 200, 3);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   // bounds

   for (size_t nb = 0; nb < bodies.size(); nb++)
   {
      world.addBody(bodies[nb]);
   }
}

void stack_init()
{
   world.init();

   double angle_vel = 0;
   std::vector<Body> bodies;

   Body body = Body(&(rectangle()), 0, 0, 0, 0);
   rectangle* rect;

   // lower bound
   rect = new rectangle(0, -14, 0, 200, 4);
   body = Body(rect, 100000, 0, 0, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(0, -10, 0, 4, 4);
   body = Body(rect, 16, 0, 0, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(0, -7, 0, 8, 2);
   body = Body(rect, 16, 0, 0, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(0, -3, 0, 2, 6);
   body = Body(rect, 12, 0, 0, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(0.5, 2, 0, 4, 4);
   body = Body(rect, 12, 0, 0, 0);
   bodies.push_back(body);
   delete rect;

   for (size_t nb = 0; nb < bodies.size(); nb++)
   {
      world.addBody(bodies[nb]);
   }
}