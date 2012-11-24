
#include "Rope.h"

const double Rope::radius   = 0.1;
const double Rope::distance = 0.5;

Rope::Rope(Vector2 const& point, size_t num_points, double mazz, double elast, double damp)
    : points    (num_points)
    , elasticity(elast)
    , damping   (damp)
{
    Vector2 disp(point);
    double single_mass = mazz / num_points;
    for (size_t i = 0; i < num_points; ++i)
    {
        points[i] = new Body(new circle(disp.v1, disp.v2, 0, radius), single_mass, 0, 0, 0);
        disp.v2 -= distance;
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
