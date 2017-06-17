#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"

class trajectory_controller
{

    private:
        nav_msgs::Path current_path; // current path to follow
        geometry_msgs::Pose current_goal; // current goal for controller
        geometry_msgs::Pose current;

        int m; // mass
        float k; // thrust_rpm_const_k
        float b; // torque_drag_const_b

        float Ixx; // Massentraegheitsmomente
        float Iyy;
        float Izz;

        float L; // distance from COG to any one of the propellers
        float g; // 9.81 m/s²

    public:
        trajectory_controller(ros::NodeHandle nh); // constructor

        // sets current_path
        void set_path(const nav_msgs::Path::ConstPtr& msg);

        void set_current_pose(const geometry_msgs::Pose::ConstPtr& msg);

};

#endif
