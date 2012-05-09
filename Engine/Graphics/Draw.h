#ifndef INCLUDE_DRAW
#define INCLUDE_DRAW

#include <vector>
#include <deque>
#include "Body\Body.h"
#include "Collision\GJK.h"
#include "Collision\Collision.h"
#include "Constraints\Constraint.h"
#include "Constraints\DoFConstraint.h"
#include "Math\MathRoutines.h"
#include "glut/glut.h"

const double SURFACE_DISTANCE = 1000;
const double CIRCLE_SEGMENTS = 50;

void init_color();
void reshape_window(int width, int height);
void draw_bodies(const std::vector<Body>& bodies);
void draw_collisions(const std::vector<Collision>& collisions);
void draw_constraints(const std::vector<Constraint*>& constraints);
void draw_simplex(Simplex simplex);
void draw_point(Vector2 point);
void draw_segment(Segment segment);

#endif