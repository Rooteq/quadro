#pragma once
#include "inverse_kinematics.hpp"
#include "array"

#define TRAJECTORY_POINTS 6
#define DOWN_PHASES 3 //number of times an individual leg touches the ground during the whole phase

namespace IK
{

using vec3 = Eigen::Vector3d;

// Again starting from FL
typedef Eigen::Matrix<unsigned int, 4, 4> CrawlMatrix;

struct LegTrajectory
{
    std::array<vec3, TRAJECTORY_POINTS> up{};

    vec3 leg_down_offset{0.0,0.0,0.0};

    vec3 current_leg_pos;
    bool up_movement = false;
    int i = 0;
    int k = 0;

    double step_base, step_height;
    double rotation_angle_{0.0};

    void calculate_shape(const double base, const double height, const double walking_speed) // add params here
    {
        step_base = base;
        step_height = height;

        const double x_step = walking_speed * step_base/(TRAJECTORY_POINTS/2);

        double coeff = height / (base/2);

        // SET POINT, WHERE THE LEG WILL RAISE FROM
        up[0] << walking_speed*(-step_base/2), 0.0, 0.0;

        for(int i = 1; i < TRAJECTORY_POINTS; ++i)
        {
            if(i < TRAJECTORY_POINTS/2)
                up[i] << (up[i-1].x() + x_step/2) , 0.0 , (up[i-1].z() + coeff * x_step/2);
            else
                up[i] << (up[i-1].x() + x_step/2) , 0.0 , (up[i-1].z() - coeff * x_step/2);
        }

        const double x_step_down = walking_speed * step_base/(TRAJECTORY_POINTS*DOWN_PHASES); 
        leg_down_offset = {x_step_down, 0.0, 0.0};

        // Apply rotation if needed
        apply_rotation();
    }

    void set_rotation(double rotation_angle)
    {
        rotation_angle_ = rotation_angle;
        // apply_rotation();
    }

    void apply_rotation()
    {
        if (rotation_angle_ != 0.0) {
            Eigen::Matrix3d rotation;
            rotation = Eigen::AngleAxisd(rotation_angle_, Eigen::Vector3d::UnitZ());
            
            for(auto& point : up) {
                point = rotation * point;
            }

            leg_down_offset = rotation * leg_down_offset;
        }
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

    const vec3& get_next_down_step() // can add PID controler here?
    {
        current_leg_pos -= leg_down_offset;

        return current_leg_pos;
    }
};

class PositionController
{

public:
    PositionController() // Placeholder
    {
        for(Leg leg : legIterator())
        {
            legs_trajectory[leg].calculate_shape(crawl_reach, crawl_height, walking_speed_);
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

    void set_walking_parameters(const double walk_speed, const double yaw_speed, 
                               const double walking_rotation, const bool walking_enabled)
    {
        walking_speed_ = walk_speed;
        walking_yaw_speed_ = yaw_speed;
        walking_rotation_angle_ = walking_rotation;
        walking_enabled_ = walking_enabled;
        
        // This is very much temporary, robot rotation should be calculated based on a rectangle inscribed in a circle (first thought)
        // It should be possible to have walking on a plane and yaw happen at the same time!!! - needs controller redesign
        if (walking_enabled_) {
            for(Leg leg : legIterator()) {
                if(yaw_speed != 0)
                {
                    if(yaw_speed > 0)
                    {
                        if(leg == Leg::FL || leg == Leg::FR)
                        {
                            legs_trajectory[leg].set_rotation(M_PI/2);
                        }
                        else
                        {
                            legs_trajectory[leg].set_rotation(-M_PI/2);
                        }
                    }
                    else
                    {
                        if(leg == Leg::BL || leg == Leg::BR)
                        {
                            legs_trajectory[leg].set_rotation(M_PI/2);
                        }
                        else
                        {
                            legs_trajectory[leg].set_rotation(-M_PI/2);
                        }

                    }

                    walking_speed_ = std::abs(yaw_speed); // This shit is so bad
                }
                else 
                    legs_trajectory[leg].set_rotation(walking_rotation_angle_);
            }
        }
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

        if(no_up_movement() && !walking_enabled_)
        {
            return;
        }

        // Set up new step
        if(no_up_movement())
        {

            for(Leg leg : legIterator())
            {
                legs_trajectory[leg].calculate_shape(crawl_reach, crawl_height, walking_speed_);
            }

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


        for(Leg leg : legIterator())
        {
            // robot_rot.y() = 0.05;
            if(legs_trajectory[leg].up_movement)
            {
                if(leg == Leg::FL || leg == Leg::BL)
                    robot_rot.x() = walking_speed_ * -0.1;
                else
                    robot_rot.x() = walking_speed_ * 0.1;
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

    const double crawl_reach = 0.08;
    const double crawl_height = 0.04;

    vec3 legs_pos[sizeof(Leg)] = {{0.0, 0.0, 0.0}};
    
    const vec3 default_leg_pos{-crawl_reach/2, 0.0, -0.27};

    vec3 legs_pos_before_rotation[sizeof(Leg)] = {this->default_leg_pos, this->default_leg_pos, this->default_leg_pos, this->default_leg_pos};

    vec3 robot_rot{0.0, 0.0, 0.0};

    // Walking control variables
    double walking_speed_{0.0};
    double walking_yaw_speed_{0.0};
    double walking_rotation_angle_{0.0};
    bool walking_enabled_{false};

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