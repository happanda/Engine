#ifndef INCLUDE_FORCE
#define INCLUDE_FORCE

#include "Body\Body.h"
#include "Math\Vector2.h"

class Force
{
public:
    Force(void);
    // force by x, by y and torque
    double Magnitude[3];
    // body suffering from force
    Body* Body;
    // point of appliance in local coordinates
    Vector2 LocalPoint;
    double* TimeStep;
    int id;
    void Apply(void);    
};

#endif
