#include "Force.h"

Force::Force(void)
{
    Magnitude[0] = Magnitude[1] = Magnitude[2] = 0;
    Body = 0;
    // point of appliance in local coordinates
    LocalPoint = Vector2::ORIGIN;
    TimeStep = 0;
}

void Force::Apply(void)
{
    Vector2 impulse = (Vector2(1, 0) * Magnitude[0] + Vector2(0, 1) * Magnitude[1]) * (*TimeStep);
    Body->velocity = Body->velocity + impulse * Body->iMass;
    double iinrt1 = Body->iInert;
    Vector3 imp3(impulse.v1, impulse.v2, 0);
    Vector3 r3(LocalPoint.v1, LocalPoint.v2, 0);
    r3 = r3.cross(imp3);
    double ro1 = r3.v3;
    Body->angle_vel += iinrt1 * ro1;
    Body->angle_vel += iinrt1 * Magnitude[2];
}