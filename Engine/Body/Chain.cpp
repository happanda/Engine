
#include "Chain.h"

const double Chain::radius   = 0.2;
const double Chain::distance = 0.5;

Chain::Chain(Vector2 const& point, size_t num_points, double mazz, world_vars* vars)
    : points    (num_points)
    , w_vars    (vars)
{
    Vector2 disp(point);
    double single_mass = mazz / num_points;
    for (size_t i = 0; i < num_points; ++i)
    {
        points[i] = new Body(new circle(disp.v1, disp.v2, 0, radius), single_mass, 0, 0, 0);
        disp.v2  -= distance;
        if (i > 0)
        {
            constraints_.push_back(new FixedConstraint(points[i], Vector2::ORIGIN, points[i - 1],
                Vector2::ORIGIN, w_vars));
        }
    }
}

Chain::Chain(const Chain& rope)
    : points(rope.points)
{
}

const Chain& Chain::operator=(const Chain& rope)
{
    return *this;
}
