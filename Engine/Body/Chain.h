#ifndef INCLUDE_CHAIN
#define INCLUDE_CHAIN

#include <vector>
#include "Body/Body.h"
#include "Body/Shape.h"
#include "Math/Vector2.h"

struct Chain
{
    std::vector<Body*> points;
    double elasticity;
    double damping;

    Chain(Vector2 const& point, size_t num_points, double mazz, double elast, double damp);
    Chain(const Chain& rope);

    const Chain& operator=(const Chain& rope);

    static const double radius;     // radius of points
    static const double distance;   // distance between points
};

#endif