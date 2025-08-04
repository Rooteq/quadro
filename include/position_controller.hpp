#pragma once
#include "inverse_kinematics.hpp"

namespace IK
{
    

class PositionController
{
public:
public:
    const LegJointPositions& get_leg_joint_positions(Leg leg)
    {
        return legs_ik.legs[leg];
    }
    InverseKinematics legs_ik;
private:
};

} // namespace IK