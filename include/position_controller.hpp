#pragma once
#include "inverse_kinematics.hpp"
#include "array"

#define TRAJECTORY_POINTS 9
#define DOWN_PHASES 3 //number of times an individual leg touches the ground during the whole phase

namespace IK
{

using vec3 = Eigen::Vector3d;

// Again starting from FL
typedef Eigen::Matrix<unsigned int, 4, 4> CrawlMatrix;

struct LegTrajectory
{
    std::array<vec3, TRAJECTORY_POINTS> up{};
    std::array<vec3, TRAJECTORY_POINTS*DOWN_PHASES> down{};
    vec3 leg_offset_before_step;
    vec3 current_leg_pos;
    bool up_movement = false;
    int i = 0;
    int k = 0;

    double step_base, step_height;

    void calculate_shape(const double base, const double height) // add params here
    {
        step_base = base;
        step_height = height;

        const double x_step = base/(TRAJECTORY_POINTS/2);

        up[0] << 0.0, 0.0, 0.0;
        for(int i = 1; i < TRAJECTORY_POINTS; ++i)
        {
            if(i < TRAJECTORY_POINTS/2)
                up[i] << (up[i-1].x() + x_step/2) , 0.0 , (up[i-1].z() + 2.4*x_step/2);
            else
                up[i] << (up[i-1].x() + x_step/2) , 0.0 , (up[i-1].z() - 2.4*x_step/2);

            // if(i < TRAJECTORY_POINTS/3)
            //     shape[i] << (shape[i-1].x() - x_step) , 0.0 , 0.0;
            // else if(i >= int(TRAJECTORY_POINTS/3) && i < 2*(int(TRAJECTORY_POINTS/3)))
            //     shape[i] << (shape[i-1].x() + x_step/2) , 0.0 , (shape[i-1].z() + 2.4*x_step/2);
            // else
            //     shape[i] << (shape[i-1].x() + x_step/2) , 0.0 , (shape[i-1].z() - 2.4*x_step/2);
        }

        const double x_step_down = base/(TRAJECTORY_POINTS*DOWN_PHASES); 

        down[0] << 0.0, 0.0, 0.0;
        for(int i = 1; i < TRAJECTORY_POINTS*DOWN_PHASES; ++i)
        {
            down[i] << (down[i-1].x() - x_step_down), 0.0, 0.0;
        }

        leg_offset_before_step = down[TRAJECTORY_POINTS*DOWN_PHASES-1];

    }

    const vec3& get_next_up_pos()
    {
        int j = i;
        i++;

        if(i == TRAJECTORY_POINTS)
        {
            i = 0;
            up_movement = false;
        }

        current_leg_pos = up[j];
        return current_leg_pos;
    }

    const vec3& get_next_down_step()
    {
        const double x_step_down = step_base/(TRAJECTORY_POINTS*DOWN_PHASES); 

        current_leg_pos.x() -=x_step_down;

        return current_leg_pos;
        // int j = k;
        // k++;

        // if(k == TRAJECTORY_POINTS * DOWN_PHASES)
        // {
        //     k = 0;
        //     // finished = true;
        // }

        // return down[j];
    }
};

class PositionController
{

public:
    PositionController() // Placeholder
    {
        for(Leg leg : legIterator())
        {
            legs_trajectory[leg].calculate_shape(0.05, 0.04);
        }
    }

    void startup()
    {
        for(Leg leg : legIterator())
        {
            legs_ik.calcJointPositions(leg, default_leg_pos.x(), default_leg_pos.y(), default_leg_pos.z());
            legs_pos[leg] = default_leg_pos;
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
        // apply_rotation();
        apply_gait();
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
            vec3 xyz = (rotationMatrix.inverse() * (legs_pos_before_rotation[leg] + legs_origin[leg]));
            legs_pos[leg] = xyz - legs_origin[leg];
        }
    }

    void apply_gait()
    {
        

        Eigen::Vector<unsigned int, 4> leg_config = crawl_matrix.col(current_gait_phase);

        // Set up new step
        if(no_up_movement())
        {

            for(int i = 0; i < 4; ++i) //go over legs
            {
                if(leg_config[i] == 0) // This leg should draw the shape (start the motion)
                {
                    legs_trajectory[i].up_movement = true;
                }
            }

            current_gait_phase++;
            if(current_gait_phase > 3) current_gait_phase = 0; // CHECK FOR SIZE

        } 


        // Make movement
        for(Leg leg : legIterator())
        {
            robot_rot.y() = 0.05;
            if(legs_trajectory[leg].up_movement)
            {
                if(leg == Leg::FL || leg == Leg::BL)
                    robot_rot.x() = -0.1;
                else
                    robot_rot.x() = 0.1;
                legs_pos_before_rotation[leg] = legs_trajectory[leg].get_next_up_pos() + default_leg_pos;
            }
            else
                legs_pos_before_rotation[leg] = legs_trajectory[leg].get_next_down_step() + default_leg_pos;
             
        }       
    }

private:
    bool no_up_movement()    
    {
        for(Leg leg : legIterator())
        {
            if(legs_trajectory[leg].up_movement)
                return false;
        }       
        return true;
    }

    vec3 legs_pos[sizeof(Leg)] = {{0.0, 0.0, 0.0}};
    
    const vec3 default_leg_pos{0.0, 0.0, -0.30};

    vec3 legs_pos_before_rotation[sizeof(Leg)] = {this->default_leg_pos, this->default_leg_pos, this->default_leg_pos, this->default_leg_pos};

    vec3 robot_rot{0.0, 0.0, 0.0};

    vec3 legs_origin[sizeof(Leg)] = {{0.185, 0.0628, 0.0},
                                     {0.185, -0.0628, 0.0},
                                     {-0.185, 0.0628, 0.0},
                                     {-0.185, -0.0628, 0.0}};
    
    LegTrajectory legs_trajectory[sizeof(Leg)];

    const CrawlMatrix crawl_matrix{
        { 1,  0,  1,  1},
        { 1,  1,  1,  0},
        { 0,  1,  1,  1},
        { 1,  1,  0,  1}
    };

    unsigned int current_gait_phase = 0;
};

}