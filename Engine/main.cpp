#include <stdio.h>
#include <time.h>
#include <vector>
#include <cstdlib>
#include <float.h>
#include "World\World.h"
#include "Body\Shape.h"
#include "Body\Destructable.h"
#include "Body\Body.h"
#include "Graphics\Draw.h"
#include "Constraints\DoFConstraint.h"
#include "Constraints\FixedConstraint.h"
#include "Motors\DoFmotor.h"
#include "test_GJK.h"
#include "glut/glut.h"
#include "anttweakbar\AntTweakBar.h"

World world;
Body* player_body = 0;
Force* drag_force = 0;
double drag_force_spring_coef = 50;
double drag_force_damper_coef = 20;
Body* kickBody = 0;

double motion_x = 0;
double motion_y = 0;

int speed = 20;
bool pause = false;
bool draw_cos = false;
bool draw_tw = true;

void init_game();

void glut_init();
void tw_init();

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
    //"1-4: demos, p - pause, y - kick body, ' - draw contacts/constraints"
    clock_t cl = clock();
    if (cl > prevCl + (double)CLOCKS_PER_SEC * speed / 1000 && !pause)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        draw_bodies(world.bodies);
        //draw_ropes(world.ropes);
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
        if (kickBody != 0)
        {
            double wx, wy;
            screen_coords2world(motion_x, motion_y, wx, wy);
            glColor3f(0.0f, 0.0f, 1.0f);
            draw_segment(Segment(Vector2(wx, wy), kickBody->form->point));
        }

        if (player_body)
        {
            camera_xpos = player_body->form->point.v1;
            camera_ypos = player_body->form->point.v2;

            int w = glutGet(GLUT_WINDOW_WIDTH);
            int h = glutGet(GLUT_WINDOW_HEIGHT);
            reshape_window(w, h);
        }
        if (draw_tw)
            TwDraw();
        glutSwapBuffers();
    }

    if (cl > prevCl + (double)CLOCKS_PER_SEC * speed / 1000 && !pause)
    {
        world.update(world.vars.timeStep);
        prevCl = cl;
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
    if (key == '1')
        init_game();
    if (key == 'p')
        pause = !pause;
    if (key == 'q')
        exit(0);
    if (key == '\'')
        draw_cos = !draw_cos;
    if (key == 'y')
    {
        int r = rand() % world.bodies.size();
        if (world.bodies[r]->mass < world.vars.UNMOVABLE_MASS)
            world.bodies[r]->velocity = world.bodies[r]->velocity + Vector2(0, 12);
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
    for (std::vector<Body*>::iterator it = world.bodies.begin(); it != world.bodies.end(); ++it)
    {
        if (check_point_inside(p, *it, localCoord))
        {
            local = localCoord;
            return *it;
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
    if (btn == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    {
        Vector2 localCoord;
        kickBody = click_inside(x, y, localCoord);
    }
    if (btn == GLUT_RIGHT_BUTTON && state == GLUT_UP)
    {
        if (kickBody != 0)
        {
            double wx, wy;
            screen_coords2world(motion_x, motion_y, wx, wy);
            Vector2 kickImpulse = (Vector2(wx, wy) - kickBody->form->point) * 5;
            if (kickImpulse.norm2sq() > 35)
            {
                kickImpulse.normalize2();
                kickImpulse = kickImpulse * 35;
            }
            if (kickBody->mass < world.vars.UNMOVABLE_MASS)
                kickBody->velocity = kickBody->velocity + kickImpulse;
            kickBody = 0;
        }
    }
    if (btn == GLUT_LEFT_BUTTON && state == GLUT_UP)
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
}

void specialKey(int key, int x, int y)
{
    if (!player_body)
        return;

    double vel_delta = 1;
    double max_vel = 15;
    if (key == GLUT_KEY_UP)
    {
        if (player_body->velocity.v2 <= 0)
            player_body->velocity.v2 = 12;
    }
    if (key == GLUT_KEY_DOWN)
    {
    }
    if (key == GLUT_KEY_LEFT)
    {
        if (player_body->angle_vel < 0)
            player_body->angle_vel = -0;
        player_body->angle_vel += vel_delta;
    }
    if (key == GLUT_KEY_RIGHT)
    {
        if (player_body->angle_vel > 0)
            player_body->angle_vel = 0;
        player_body->angle_vel -= vel_delta;
    }

    if (player_body->angle_vel > max_vel)
        player_body->angle_vel = max_vel;
    if (player_body->angle_vel < -max_vel)
        player_body->angle_vel = -max_vel;
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

    init_game();
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

    /*glRasterPos2i(100, 120);
    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);*/
    //glutBitmapString(GLUT_BITMAP_HELVETICA_18, );

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


void init_game()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = -9.8;
    wvars.RESTITUTION = 0.3;
    wvars.FRICTION = 0.5;
    world.init();
    world.vars = wvars;

    // PLAYER
    world.addBody(new Body(new circle(0, 0, 0, 2), 10, 0, 0, 0));
    player_body = &(*world.bodies.front());

    double bigmass = wvars.UNMOVABLE_MASS;
    world.addBody(new Body(new rectangle(0, -14, 0, 200, 3), bigmass, 0, 0, 0));


    //Destructable* destr = new Destructable(Vector2::ORIGIN, 2, 2, 100, &world.vars);
    //world.addDestructable(destr);
}
