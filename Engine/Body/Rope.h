#ifndef INCLUDE_ROPE
#define INCLUDE_ROPE

#include <vector>
#include "Body/Body.h"
#include "Body/Shape.h"
#include "Math/Vector2.h"

enum orientation_t
{
    o_vertical,
    o_horizontal
};

struct Rope
{
    std::vector<Body*> points;
    double elasticity;
    double damping;

    Rope(Vector2 const& point, size_t num_points, double mazz, double elast, double damp,
        orientation_t orient);
    Rope(const Rope& rope);

    const Rope& operator=(const Rope& rope);

    static const double radius;     // radius of points
    static const double distance;   // distance between points
};

#endif