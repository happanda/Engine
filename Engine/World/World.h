#ifndef INCLUDE_WORLD
#define INCLUDE_WORLD

#include <vector>
#include "WorldVars.h"
#include "Body\Body.h"
#include "Body\Rope.h"
#include "Force\Force.h"
#include "Collision\Collision.h"
#include "Constraints\Constraint.h"

class World
{
public:
    static const size_t FORCE_DIM = 6;
    World();
    void init();
    void addBody(Body* body);
    void addRope(Rope* rope);
    void addConstraint(Constraint* constraint);
    void addForce(Force force);
    void removeForce(int id);
    void update(double deltaT);
    void apply_forces(double deltaT);
    void resolve_collision(double deltaT);
    void resolve_constraints(double deltaT);

    std::vector<Collision*> collisions;
    std::vector<Body*> bodies;
    std::vector<Rope*> ropes;
    std::vector<Constraint*> constraints;
    std::vector<Force> forces;
    world_vars vars;

    void resolve_collision_deprecated(double deltaT);
private:
    double force_ext[FORCE_DIM];
    int force_id;
};

#endif
