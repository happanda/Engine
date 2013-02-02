#include "Destructable.h"

Destructable::Destructable(Vector2 const& point, size_t width, size_t height, double mazz, world_vars* vars)
    : w_vars    (vars)
{
    part_width  = 2;
    part_height = 2;

    size_t num_parts = width * height;
    double single_mass = mazz / num_parts;

    for (size_t i = 0; i < width; ++i)
    {
        parts.push_back(std::vector<Body*>());
        for (size_t j = 0; j < height; ++j)
        {
            parts[i].push_back(new Body(new rectangle(point.v1 + i * (part_width + 0.01), point.v2 + j * (part_height + 0.01),
                0, part_width, part_height), single_mass, 0, 0, 0));
        }
    }

    for (size_t i = 0; i < width; ++i)
        for (size_t j = 0; j < height - 1; ++j)
        {
            constraints.push_back(new FixedConstraint(parts[i][j], Vector2(part_width / 2, part_height / 2),
                parts[i][j + 1], Vector2(part_width / 2, -part_height / 2), w_vars));
            constraints.push_back(new FixedConstraint(parts[i][j], Vector2(-part_width / 2, part_height / 2),
                parts[i][j + 1], Vector2(-part_width / 2, -part_height / 2), w_vars));
        }

    for (size_t i = 0; i < width - 1; ++i)
        for (size_t j = 0; j < height; ++j)
        {
            constraints.push_back(new FixedConstraint(parts[i][j], Vector2(part_width / 2, part_height / 2),
                parts[i + 1][j], Vector2(-part_width / 2, part_height / 2), w_vars));
            constraints.push_back(new FixedConstraint(parts[i][j], Vector2(part_width / 2, -part_height / 2),
                parts[i + 1][j], Vector2(-part_width / 2, -part_height / 2), w_vars));
        }
}

Destructable::Destructable(const Destructable& destr)
{
}

Destructable& Destructable::operator=(const Destructable& destr)
{
    return *this;
}