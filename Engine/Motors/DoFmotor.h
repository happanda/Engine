#ifndef INCLUDE_DOF_MOTOR
#define INCLUDE_DOF_MOTOR

#include "Constraints/DoFConstraint.h"

enum DoFmotorType
{
    STABLE          = 0x000,
    MOVE_X          = 0x001,
    MOVE_Y          = 0x002,
    MOVE_XY         = 0x003,
    ROTATE          = 0x004,
    MOVE_X_ROTATE   = 0x005,
    MOVE_Y_ROTATE   = 0x006,
    MOVE_XY_ROTATE  = 0x007
};

class DoFmotor
    : public DoFConstraint
{
public:
    DoFmotor(Body* body, DoFmotorType type, world_vars* vars);
    DoFmotorType motor_type;
    void SetMotorBias(std::vector<double> const& dzeta);
    void SetMotorLimits(double lower_limit, double upper_limit);
protected:
    void _deltaImpulse(Vector2& impulse, double& torque);
private:
    void init();
    double              lower_lambda;
    double              upper_lambda;
    std::vector<double> motor_bias_;
};

#endif