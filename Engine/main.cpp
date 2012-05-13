#include <stdio.h>
#include <time.h>
#include <vector>
#include <cstdlib>
#include <float.h>
#include "World\World.h"
#include "Body\Shape.h"
#include "Body\Body.h"
#include "Graphics\Draw.h"
#include "Constraints\DoFConstraint.h"
#include "test_GJK.h"
#include "glut/glut.h"
#include "anttweakbar\AntTweakBar.h"

World world;
Force* drag_force = 0;
double drag_force_spring_coef = 20;
double drag_force_damper_coef = 10;
double motion_x = 0;
double motion_y = 0;

int speed = 20;
bool pause = false;
bool draw_cos = false;
bool draw_tw = true;

void init_bodies();
void init_many_rectangles();
void init_many_circles();
void glut_init();
void tw_init();
void stack_init();

extern double zoom_distance;
extern double camera_xpos;
extern double camera_ypos;
extern double viewfield_minx;
extern double viewfield_miny;
extern double viewfield_maxx;
extern double viewfield_maxy;

void screen_coords2world(int x, int y, double& wx, double& wy)
{
    int w = glutGet(GLUT_WINDOW_WIDTH);
    int h = glutGet(GLUT_WINDOW_HEIGHT);
    y = h - y;
    wx = viewfield_minx + (viewfield_maxx - viewfield_minx) * ((double)x / w);
    wy = viewfield_miny + (viewfield_maxy - viewfield_miny) * ((double)y / h);
}

clock_t prevCl = 0;
void step()
{
    clock_t cl = clock();
    if (cl > prevCl + (double)CLOCKS_PER_SEC * speed / 1000 && !pause)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        draw_bodies(world.bodies);
        if (draw_cos)
        {
            draw_collisions(world.collisions);
            draw_constraints(world.constraints);
        }

        if (drag_force != 0)
        {
            double wx, wy;
            screen_coords2world(motion_x, motion_y, wx, wy);
            Vector2 head(wx, wy), tail(drag_force->Body->form->point + drag_force->LocalPoint);
            Vector2 vect = head - tail;
            drag_force->Magnitude[0] = drag_force_spring_coef * vect.v1 * drag_force->Body->mass
                - drag_force_damper_coef * drag_force->Body->velocity.v1;
            drag_force->Magnitude[1] = drag_force_spring_coef * vect.v2 * drag_force->Body->mass
                - drag_force_damper_coef * drag_force->Body->velocity.v2;
            glColor3f(0.0f, 1.0f, 0.0f);
            draw_segment(Segment(head, tail));
        }

        world.update(world.vars.timeStep);
        prevCl = cl;
        if (draw_tw)
            TwDraw();
        glutSwapBuffers();
    }
}

void reshape(int width, int height)
{
    reshape_window(width, height);
    //glutReshapeWindow(width, height);
    if (draw_tw)
        TwWindowSize(width, height);
}

void keyboard(unsigned char key, int x, int y)
{
    if (key == 'p')
        pause = !pause;
    if (key == 'q')
        exit(0);
    if (key == '\'')
        draw_cos = !draw_cos;
    if (key == 'y')
    {
        int r = rand() % world.bodies.size();
        if (world.bodies.at(r).mass < world.vars.UNMOVABLE_MASS)
            world.bodies.at(r).velocity = world.bodies.at(r).velocity + Vector2(0, 12);
    }
    if (key == '+' || key == '-')
    {
        int w = glutGet(GLUT_WINDOW_WIDTH);
        int h = glutGet(GLUT_WINDOW_HEIGHT);
        if (key == '+')
            zoom_distance -= 1;
        if (key == '-')
            zoom_distance += 1;
        reshape(w, h);
    }
    if (key == '1') { glut_init(); init_bodies(); tw_init(); draw_tw = true; }
    //if (key == '0') { test_gjk_init(); draw_tw = false; }
    if (key == '2') { glut_init(); stack_init(); tw_init(); draw_tw = true; }
    if (key == '3') { glut_init(); init_many_rectangles(); tw_init(); draw_tw = true; }
    if (key == '4') { glut_init(); init_many_circles(); tw_init(); draw_tw = true; }
    if (draw_tw)
        TwEventKeyboardGLUT(key, x, y);
    glutBitmapCharacter(GLUT_BITMAP_8_BY_13, key);
    glutPostRedisplay();
}

Body* click_inside(int x, int y, Vector2& local)
{
    double cursor_xpos;
    double cursor_ypos;
    screen_coords2world(x, y, cursor_xpos, cursor_ypos);
    Vector2 localCoord;
    const Vector2 p(cursor_xpos, cursor_ypos);
    for (std::vector<Body>::iterator it = world.bodies.begin(); it != world.bodies.end(); it++)
    {
        if (check_point_inside(p, &(*it), localCoord))
        {
            local = localCoord;
            return &(*it);
        }
    }
    return 0;
}

void mouse(int btn, int state, int x, int y)
{
    motion_x = x;
    motion_y = y;
    TwEventMouseButtonGLUT(btn, state, x, y);
    int w = glutGet(GLUT_WINDOW_WIDTH);
    int h = glutGet(GLUT_WINDOW_HEIGHT);
    if (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        Vector2 localCoord;
        Body* body = click_inside(x, y, localCoord);
        if (body != 0)
        {
            Force force;
            force.Body = body;
            force.LocalPoint = localCoord;
            world.addForce(force);
            drag_force = &world.forces.at(world.forces.size() - 1);
            printf("Inside body!!\n");
        }
    }
    /*if (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        Vector2 localCoord;
        Body* body = click_inside(x, y, localCoord);
        if (body != 0)
        {
            Force force;
            force.Body = body;
            force.LocalPoint = localCoord;
            force.Magnitude[2] = 5;
            world.addForce(force);
            printf("Inside body!!\n");
        }
    }
    if (btn == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    {
        Vector2 localCoord;
        Body* body = click_inside(x, y, localCoord);
        if (body != 0)
        {
            Force force;
            force.Body = body;
            force.LocalPoint = localCoord;
            force.Magnitude[2] = -5;
            world.addForce(force);
            printf("Inside body!!\n");
        }
    }*/
    if (state == GLUT_UP)
    {
        drag_force = 0;
        world.forces.clear();
    }
}

void motion(int x, int y)
{
    motion_x = x;
    motion_y = y;
    TwEventMouseMotionGLUT(x, y);
}

void passiveMotion(int x, int y)
{
    motion_x = x;
    motion_x = y;
    TwEventMouseMotionGLUT(x, y);
}

void choice_selected(int value)
{
    if (value == 1) { glut_init(); init_bodies(); tw_init(); draw_tw = true; }
    if (value == 2) { test_gjk_init(); draw_tw = false; }
    if (value == 3) { glut_init(); stack_init(); tw_init(); draw_tw = true; }
    if (value == 4) { glut_init(); init_many_rectangles(); tw_init(); draw_tw = true; }
    if (value == 5) { glut_init(); init_many_circles(); tw_init(); draw_tw = true; }
}

void specialKey(int key, int x, int y)
{
    if (key == GLUT_KEY_UP)
    {
        world.vars.timeStep += 0.003;
    }
    if (key == GLUT_KEY_DOWN)
    {
        if (abs(world.vars.timeStep) > 0.006)
            world.vars.timeStep -= 0.003;
    }
    world.vars.iTimeStep = 1 / world.vars.timeStep;
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

    /*glutCreateMenu(choice_selected);
    glutAddMenuEntry("Main simulation", 1);
    glutAddMenuEntry("Test GJK", 2);
    glutAddMenuEntry("Stack boxes", 3);
    glutAddMenuEntry("Many rectangles", 4);
    glutAddMenuEntry("Many circles", 5);
    glutAttachMenu(GLUT_RIGHT_BUTTON);*/
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
    wvars.GRAVITATION.v2 = -9.8;
    //wvars.RESTITUTION = 1;
    world.init();
    world.vars = wvars;
    double angle_vel = 0;

    world.addBody(Body(new rectangle(0, 0, 2, 4, 4), 16, 0, 0, 0));
    world.addBody(Body(new rectangle(0, 5, -1, 2, 2), 4, 0, -2, 0));
    world.addBody(Body(new rectangle(-5, 0, 0, 2, 2), 4, 1, 0, angle_vel));
    world.addBody(Body(new rectangle(0, -5, 0, 2, 2), 4, 0, 1, 0));
    world.addBody(Body(new rectangle(5, 0, 0, 2, 2), 4, -1, 0, 0));
    world.addBody(Body(new rectangle(10, 5, 0, 3, 1), 3, -3, -3, 0));
    world.addBody(Body(new rectangle(-9, -6, 0.2, 1, 10), 10, 2, 0, 0));
    world.addBody(Body(new circle(-3, 4, 0, 2), 2, 0, -4, 0));
    world.addBody(Body(new circle(4, 7, 0, 3), 8, -2, -2, 0));

    // bounds
    double bigmass = 100000;
    world.addBody(Body(new rectangle(-40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(0, 14, 0, 200, 3), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(0, -14, 0, 200, 3), bigmass, 0, 0, 0));
    // bounds

    // some simple axis constraints
    world.addConstraint(new DoFConstraint(&world.bodies[0], Y_ANGLE, &(world.vars)));
    world.addConstraint(new DoFConstraint(&world.bodies[2], XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(&world.bodies[3], XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(&world.bodies[4], XY_AXIS, &(world.vars)));
}

void init_many_rectangles()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = 0;
    //wvars.RESTITUTION = 1;
    world.init();
    world.vars = wvars;
    double angle_vel = 0;

    int s = 12;
    double sp = (double)RAND_MAX / 12;
    for (int i = -s; i < s; i += 4)
    {
        for (int j = -s + 2; j < s; j += 4)
        {
            double r = rand() % 2 + 1;
            world.addBody(Body(new rectangle(i, j, (double)rand() / sp, r, 3 - r),
                1, (double)rand() / sp - (double)rand() / sp,
            (double)rand() / sp - (double)rand() / sp, 0));
        }
    }

    // bounds
    double bigmass = 100000;
    world.addBody(Body(new rectangle(-40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(0, 14, 0, 200, 3), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(0, -14, 0, 200, 3), bigmass, 0, 0, 0));
    // bounds
}

void init_many_circles()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = 0;
    //wvars.RESTITUTION = 1;
    world.init();
    world.vars = wvars;
    double angle_vel = 0;

    int s = 12;
    double sp = (double)RAND_MAX / 12;
    for (int i = -s; i < s; i += 4)
    {
        for (int j = -s + 2; j < s; j += 4)
        {
            double r = rand() % 2 + 1;
            world.addBody(Body(new circle(i, j, (double)rand(), 1), 3.14,
                (double)rand() / sp - (double)rand() / sp,
                (double)rand() / sp - (double)rand() / sp, 0));
        }
    }

    // bounds
    double bigmass = 100000;
    world.addBody(Body(new rectangle(-40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(0, 14, 0, 200, 3), bigmass, 0, 0, 0));
    world.addBody(Body(new rectangle(0, -14, 0, 200, 3), bigmass, 0, 0, 0));
    // bounds
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