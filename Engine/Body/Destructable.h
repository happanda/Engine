#ifndef INCLUDE_DESTRUCTABLE
#define INCLUDE_DESTRUCTABLE

#include <vector>
#include "Body/Body.h"
#include "Body/Shape.h"
#include "Constraints/FixedConstraint.h"
#include "Math/Vector2.h"
#include "World/WorldVars.h"

struct Destructable
{
    Destructable(Vector2 const& point, size_t width, size_t height, double mazz, world_vars* vars);
    Destructable(const Destructable& destr);
    Destructable& operator=(const Destructable& rope);


    std::vector<std::vector<Body*> > parts;
    double part_width;
    double part_height;

    std::vector<FixedConstraint*> constraints;
    world_vars* w_vars;
};

#endif