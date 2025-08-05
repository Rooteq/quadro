#pragma once
#include "inverse_kinematics.hpp"
#include "array"

#define TRAJECTORY_POINTS 21

namespace IK
{

using vec3 = Eigen::Vector3d;

// Again starting from FL
typedef Eigen::Matrix<unsigned int, 4, 8> CrawlMatrix;

struct LegTrajectory
{
    std::array<vec3, TRAJECTORY_POINTS> shape{};
    bool started = false;
    bool finished = false;
    int i = 0;

    void calculate_shape(const double base, const double height) // add params here
    {
        const double x_step = base/(TRAJECTORY_POINTS/3);

        shape[0] << 0.0, 0.0, 0.0;
        for(int i = 1; i < TRAJECTORY_POINTS; ++i)
        {
            if(i < TRAJECTORY_POINTS/3)
                shape[i] << (shape[i-1].x() - x_step) , 0.0 , 0.0;
            else if(i >= int(TRAJECTORY_POINTS/3) && i < 2*(int(TRAJECTORY_POINTS/3)))
                shape[i] << (shape[i-1].x() + x_step/2) , 0.0 , (shape[i-1].z() + 2.4*x_step/2);
            else
                shape[i] << (shape[i-1].x() + x_step/2) , 0.0 , (shape[i-1].z() - 2.4*x_step/2);
        }
    }

    const vec3& get_next_pos()
    {
        int j = i;
        i++;

        if(i == TRAJECTORY_POINTS)
        {
            i = 0;
            finished = true;
        }

        return shape[j];
    }
};

class PositionController
{

public:
    PositionController() // Placeholder
    {
        for(Leg leg : legIterator())
        {
            legs_trajectory[leg].calculate_shape(0.05, 0.06);
        }
    }

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
        // apply_rotation();
        apply_gait();
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
    }

    void apply_gait()
    {
        

        // Startup - can be done with new step below
        if(none_started())
        {
            Eigen::Vector<unsigned int, 4> leg_config = crawl_matrix.col(current_gait_phase);

            for(int i = 0; i < 4; ++i) //go over legs
            {
                if(leg_config[i] == 0) // This leg should draw the shape (start the motion)
                {
                    legs_trajectory[i].started = true;
                }
                if(leg_config[i] == 1) // Leg should keep touching ground, so is finished
                {
                    legs_trajectory[i].started = true;
                    legs_trajectory[i].finished = true;
                }
            }

        }

        // Set up new step
        if(all_started_and_finished())
        {
            current_gait_phase++;
            if(current_gait_phase > 7) current_gait_phase = 0; // CHECK FOR SIZE
            Eigen::Vector<unsigned int, 4> leg_config = crawl_matrix.col(current_gait_phase);

            // reset legs - necessary?
            for(Leg leg : legIterator())
            {
                legs_trajectory[leg].started = false;
                legs_trajectory[leg].finished = false;
            }       

            for(int i = 0; i < 4; ++i) //go over legs
            {
                if(leg_config[i] == 0) // This leg should draw the shape (start the motion)
                {
                    legs_trajectory[i].started = true;
                }
                if(leg_config[i] == 1) // Leg should keep touching ground, so is finished
                {
                    legs_trajectory[i].started = true;
                    legs_trajectory[i].finished = true;
                }
            }

        } 


        // Make movement
        for(Leg leg : legIterator())
        {
            if(legs_trajectory[leg].started && !legs_trajectory[leg].finished)
                legs_pos[leg] = legs_trajectory[leg].get_next_pos() + default_leg_pos;
        }       
    }

private:
    bool all_started_and_finished()    
    {
        for(Leg leg : legIterator())
        {
            if(legs_trajectory[leg].started && !legs_trajectory[leg].finished)
                return false;
        }       
        return true;
    }

    bool none_started()
    {
        for(Leg leg : legIterator())
        {
            if(legs_trajectory[leg].started || legs_trajectory[leg].finished)
                return false;
        }       
        return true;
    }

    vec3 legs_pos[sizeof(Leg)] = {{0.0, 0.0, 0.0}};

    const vec3 default_leg_pos{0.0, 0.0, -0.25};
    vec3 robot_rot{0.0, 0.0, 0.0};

    vec3 legs_origin[sizeof(Leg)] = {{0.185, 0.0628, 0.0},
                                     {0.185, -0.0628, 0.0},
                                     {-0.185, 0.0628, 0.0},
                                     {-0.185, -0.0628, 0.0}};
    
    LegTrajectory legs_trajectory[sizeof(Leg)];

    const CrawlMatrix crawl_matrix{
        {1, 1, 1, 0, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 0},
        {1, 0, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 0, 1, 1}
    };

    unsigned int current_gait_phase = 0;
};

}