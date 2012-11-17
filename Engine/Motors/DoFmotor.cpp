
#include "DoFmotor.h"
#include "Math\PGSsolver.h"

DoFmotor::DoFmotor(Body* body, DoFmotorType type, world_vars* vars)
    : DoFConstraint(body, NONE, vars), motor_type(type)
    , lower_lambda(0), upper_lambda(0)
{
    DoFType dof_type;
    switch (motor_type)
    {
    case MOVE_X:
        dof_type = X_AXIS;
        break;
    case MOVE_Y:
        dof_type = Y_AXIS;
        break;
    case MOVE_XY:
        dof_type = XY_AXIS;
        break;
    case ROTATE:
        dof_type = ANGLE;
        break;
    case MOVE_X_ROTATE:
        dof_type = X_ANGLE;
        break;
    case MOVE_Y_ROTATE:
        dof_type = Y_ANGLE;
        break;
    case MOVE_XY_ROTATE:
        dof_type = XY_ANGLE;
        break;
    }
    ChangeConstraint(dof_type);

    motor_bias_.push_back(0);
    motor_bias_.push_back(0);
    motor_bias_.push_back(0);
}

void DoFmotor::SetMotorBias(std::vector<double> const& dzeta)
{
    motor_bias_ = dzeta;
}

void DoFmotor::init()
{
    DoFConstraint::init();
    if ((motor_type & MOVE_X) != 0)
        Eta[0] += w_vars->iTimeStep * motor_bias_[0];
    if ((motor_type & MOVE_Y) != 0)
        Eta[1] += w_vars->iTimeStep * motor_bias_[1];
    if ((motor_type & ROTATE) != 0)
        Eta[2] += w_vars->iTimeStep * motor_bias_[2];
}

void DoFmotor::_deltaImpulse(Vector2& impulse, double& torque)
{
    init();
    if (!Enough() || motor_type != STABLE)
    {
        SolveLambda(A, Eta, Lambda, lower_lambda, upper_lambda);
        impulse = Vector2(1, 0) * Lambda[0] + Vector2(0, 1) * Lambda[1];
        torque = Lambda[2];
        /*if (impulse + sum_impulse < 0)
        impulse = -sum_impulse;
        sum_impulse += impulse;*/
    }
    else
    {
        impulse = Vector2::ORIGIN;
    }
}

void DoFmotor::SetMotorLimits( double lower_limit, double upper_limit )
{
    lower_lambda = lower_limit;
    upper_lambda = upper_limit;
}
