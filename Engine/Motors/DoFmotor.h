#ifndef INCLUDE_DOF_MOTOR
#define INCLUDE_DOF_MOTOR

#include "Constraints/DoFConstraint.h"

enum DoFmotorType
{
    STABLE          = 0,
    MOVE_X          = 1,
    MOVE_Y          = 2,
    MOVE_XY         = 3,
    ROTATE          = 4,
    MOVE_X_ROTATE   = 5,
    MOVE_Y_ROTATE   = 6,
    MOVE_XY_ROTATE  = 7
};

class DoFmotor
    : public DoFConstraint
{
public:
    DoFmotor(Body* body, DoFmotorType type, world_vars* vars);
    DoFmotorType motor_type;
    void SetMotorBias(std::vector<double> const& dzeta);
    void SetMotorAcceleration(double lower_limit, double upper_limit);
    void SetMotorLimits(std::vector<double> const& max_speed);
protected:
    void _deltaImpulse(Vector2& impulse, double& torque);
    bool Enough(void) const;
private:
    void init();
    void Fix ();
    double              lower_lambda;
    double              upper_lambda;
    std::vector<double> motor_bias_;
    std::vector<double> max_speed;
};

#endif