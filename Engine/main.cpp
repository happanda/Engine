#include <stdio.h>
#include <time.h>
#include <vector>
#include "world.h"
#include "shape.h"
#include "body.h"
#include "draw.h"
#include "test_gjk.h"
#include "glut/glut.h"


World world = World();
int speed = 20;
bool pause = false;

void init_bodies();
void main_init();

clock_t prevCl = 0;
void step()
{
   clock_t cl = clock();
   if (cl > prevCl + (double)CLOCKS_PER_SEC * speed / 1000 && !pause)
   {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      draw_bodies(world.bodies);
      draw_collisions(world.collisions);
      world.update((double)world.timeStep / 1000);
      draw_point(Vector2(0, 0));
      prevCl = cl;
      glutSwapBuffers();
   }
}

void keyboard(unsigned char key, int x, int y)
{
   if (key == 'p')
      pause = !pause;
   if (key == 'r')
      init_bodies();
   if (key == 'q')
      exit(0);
   printf("key ‘%c’ pressed at (%d,%d)\n",
      key, x, y);
   glutBitmapCharacter(GLUT_BITMAP_8_BY_13, key);
   glutPostRedisplay();
}

void mouse(int btn, int state, int x, int y)
{
   //printf("button %d is %s at (%d,%d)\n", btn, state == GLUT_DOWN ? "down" : "up", x, y);
}

void motion(int x, int y)
{
   //printf("button motion at (%d,%d)\n", x, y);
}

void passiveMotion(int x, int y)
{
   //printf("passive motion at (%d,%d)\n", x, y);
}

void choice_selected(int value)
{
   if (value == 1) main_init();
   if (value == 2) test_gjk_init();
}

void specialKey(int key, int x, int y)
{
   if (key == GLUT_KEY_UP)
   {
      world.timeStep += 5;
   }
   if (key == GLUT_KEY_DOWN)
   {
      world.timeStep -= 5;
   }
}

int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitWindowSize(600, 600);
   glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
   glutCreateWindow("Engine");

   main_init();

   init_screen();
   glutMainLoop();
}

void main_init()
{
   pause = false;
   init_bodies();
   glutDisplayFunc(step);
   glutIdleFunc(step);
   glutKeyboardFunc(keyboard);
   glutMouseFunc(mouse);
   glutMotionFunc(motion);
   glutPassiveMotionFunc(passiveMotion);
   glutSpecialFunc(specialKey);

   glutCreateMenu(choice_selected);
   glutAddMenuEntry("Main simulation", 1);
   glutAddMenuEntry("Test GJK", 2);
   glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void init_bodies()
{
   world.init();

   double angle_vel = 0;
   std::vector<Body> bodies;

   Body body = Body(&(rectangle()), 0, 0, 0, 0);
   circle* circ;
   rectangle* rect;
   //surface* surf;

   rect = new rectangle(0, 0, 0, 4, 4);
   body = Body(rect, 4, 0, 0, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(0, 5, 0, 2, 2);
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
   body = Body(rect, 4, -3, -3, 0);
   bodies.push_back(body);
   delete rect;

   rect = new rectangle(-8, -5, 0, 1, 10);
   body = Body(rect, 4, 3, 3, 0);
   bodies.push_back(body);
   delete rect;

   // bounds
   double bigmass = 100000;
   rect = new rectangle(-12, 0, 0, 1, 22);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   rect = new rectangle(12, 0, 0, 1, 22);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   rect = new rectangle(0, 12, 0, 22, 1);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   rect = new rectangle(0, -12, 0, 22, 1);
   body = Body(rect, bigmass, 0, 0, 0);
   bodies.push_back(body);
   delete rect;
   // bounds

   for (size_t nb = 0; nb < bodies.size(); nb++)
   {
      world.addBody(bodies[nb]);
   }
}