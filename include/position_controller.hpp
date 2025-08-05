#pragma once
#include "inverse_kinematics.hpp"

namespace IK
{

using vec3 = Eigen::Vector3d;

class PositionController
{

public:
    void startup(double x, double y, double z)
    {
        for(Leg leg : legIterator())
        {
            legs_ik.calcJointPositions(leg, x, y, z);
            legs_pos[leg].x() = x;
            legs_pos[leg].y() = y;
            legs_pos[leg].z() = z;
        }
    }

    void set_leg_xyz_position(Leg leg, double x, double y, double z)
    {
        legs_ik.calcJointPositions(leg, x, y, z);
        legs_pos[leg] << x,y,z;
    }

    const LegJointPositions& get_leg_joint_positions(Leg leg)
    {
        return legs_ik.legs[leg];
    }

    void set_rotation(const double roll, const double pitch, const double yaw)
    {
        robot_rot.x() = roll;
        robot_rot.y() = pitch;
        robot_rot.z() = yaw;
    }

    void apply_control()
    {
        apply_rotation();
        calculate_joint_positions();
    }
    
public:
    InverseKinematics legs_ik;

private:

    void calculate_joint_positions()
    {
        for(Leg leg : legIterator())
        {
            legs_ik.calcJointPositions(leg, legs_pos[leg].x(), legs_pos[leg].y(), legs_pos[leg].z());
        }
    }

    void apply_rotation()
    {
        Eigen::AngleAxisd rollAngle(robot_rot.x(), vec3::UnitX());
        Eigen::AngleAxisd pitchAngle(robot_rot.y(), vec3::UnitY());
        Eigen::AngleAxisd yawAngle(robot_rot.z(), vec3::UnitZ());

        Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

        Eigen::Matrix3d rotationMatrix = q.matrix();

        for(Leg leg : legIterator())
        {
            vec3 xyz = (rotationMatrix.inverse() * (default_leg_pos + legs_origin[leg]));
            legs_pos[leg] = xyz - legs_origin[leg];
        }

        // Eigen::Vector3d br_xyz = rotationMatrix.inverse() * (default_leg_pos + br_leg_origin);
        // Eigen::Vector3d fr_xyz = rotationMatrix.inverse() * (default_leg_pos + fr_leg_origin);
        // Eigen::Vector3d bl_xyz = rotationMatrix.inverse() * (default_leg_pos + bl_leg_origin);
        // Eigen::Vector3d fl_xyz = rotationMatrix.inverse() * (default_leg_pos + fl_leg_origin);

        // Eigen::Vector3d br_xyz_bis = br_xyz - br_leg_origin;
        // Eigen::Vector3d bl_xyz_bis = bl_xyz - bl_leg_origin;
        // Eigen::Vector3d fr_xyz_bis = fr_xyz - fr_leg_origin;
        // Eigen::Vector3d fl_xyz_bis = fl_xyz - fl_leg_origin;
    }

    vec3 legs_pos[sizeof(Leg)] = {{0.0, 0.0, 0.0}};

    const vec3 default_leg_pos{0.0, 0.0, -0.25};
    vec3 robot_rot{0.0, 0.0, 0.0};

    vec3 legs_origin[sizeof(Leg)] = {{0.185, 0.0628, 0.0},
                                     {0.185, -0.0628, 0.0},
                                     {-0.185, 0.0628, 0.0},
                                     {-0.185, -0.0628, 0.0}};
};

}