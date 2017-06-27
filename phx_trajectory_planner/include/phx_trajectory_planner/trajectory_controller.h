#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"
#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "math.h"

class trajectory_controller
{

    private:
        nav_msgs::Path _current_path; // current path to follow
        geometry_msgs::Pose _current_goal; // current goal for controller
        geometry_msgs::Pose _current;

        int _m; // mass
        float _k; // thrust_rpm_const_k
        float _b; // torque_drag_const_b

        float _Ixx; // Massentraegheitsmomente
        float _Iyy;
        float _Izz;

        float _L; // distance from COG to any one of the propellers
        float _g; // 9.81 m/s²

        float _e_theta;
        float _e_phi;
        float _e_psi;

        float _theta;
        float _phi;
        float _psi;

        float _last_theta;
        float _last_phi;
        float _last_psi;

        float _theta_dot;
        float _phi_dot;
        float _psi_dot;

        float _t_last;
        float _t_current;
        float _dt;

        float _K_P;
        float _K_I;
        float _K_D;

    public:
        trajectory_controller(ros::NodeHandle nh); // constructor

        // sets current_path
        void path_callback(const nav_msgs::Path::ConstPtr& msg);

        void set_current_pose(const geometry_msgs::Pose::ConstPtr& msg);
        void set_current_goal(const geometry_msgs::Pose::ConstPtr& msg);
        void calc_controller_error();
	void transform_quaternion();
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
};

#endif
