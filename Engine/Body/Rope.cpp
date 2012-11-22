
#include "Rope.h"

const double Rope::radius   = 0.04;
const double Rope::distance = 0.1;

Rope::Rope(Vector2 const& point, size_t num_points, double mazz, double elast, double damp)
    : points    (num_points)
    , elasticity(elast)
    , damping   (damp)
{
    Vector2 disp(0, -distance);
    for (size_t i = 0; i < num_points; ++i)
    {
        points.push_back(new Body(new circle(disp.v1, disp.v2, 0, radius), mazz, 0, 0, 0));
    }
}

Rope::Rope(const Rope& rope)
    : points(rope.points)
    , elasticity(rope.elasticity)
    , damping   (rope.damping)
{
}

const Rope& Rope::operator=(const Rope& rope)
{
    elasticity = rope.elasticity;
    damping    = rope.damping;
    points     = rope.points;
    return *this;
}
