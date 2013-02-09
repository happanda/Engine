#ifndef INCLUDE_CHAIN
#define INCLUDE_CHAIN

#include <vector>
#include "Body/Body.h"
#include "Body/Shape.h"
#include "Rope.h"
#include "Constraints/FixedConstraint.h"
#include "Math/Vector2.h"
#include "World/WorldVars.h"

struct Chain
{
    std::vector<Body*> points;

    Chain(Vector2 const& point, size_t num_points, double mazz, world_vars* vars, orientation_t orient);
    Chain(const Chain& rope);

    const Chain& operator=(const Chain& rope);

    static const double radius;     // radius of points
    static const double distance;   // distance between points

    std::vector<FixedConstraint*> constraints_;

    world_vars* w_vars;
};

#endif