
#include "Chain.h"

const double Chain::radius   = 0.2;
const double Chain::distance = 0.45;

Chain::Chain(Vector2 const& point, size_t num_points, double mazz, double elast, double damp)
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

Chain::Chain(const Chain& rope)
    : points(rope.points)
    , elasticity(rope.elasticity)
    , damping   (rope.damping)
{
}

const Chain& Chain::operator=(const Chain& rope)
{
    elasticity = rope.elasticity;
    damping    = rope.damping;
    points     = rope.points;
    return *this;
}
