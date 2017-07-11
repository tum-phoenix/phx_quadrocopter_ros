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
#include "phx_uart_msp_bridge/Motor.h"
//#include "roslib.h"

class trajectory_controller
{

    private:
        nav_msgs::Path _current_path; // current path to follow
        geometry_msgs::Pose _current_goal; // current goal for controller
        geometry_msgs::Pose _current;

        double _m; // mass
        double _k; // thrust_rpm_const_k
        double _b; // torque_drag_const_b

        double _Ixx; // Massentraegheitsmomente
        double _Iyy;
        double _Izz;

        double _L; // distance from COG to any one of the propellers
        double _g; // 9.81 m/s²

        double _e_theta;
        double _e_phi;
        double _e_psi;

        double _cmd_p;
        double _cmd_q;
        double _cmd_r;

        double _theta;
        double _phi;
        double _psi;

        double _last_theta;
        double _last_phi;
        double _last_psi;

        double _integral_theta;
        double _integral_phi;
        double _integral_psi;
        double _integral_theta_PI;
        double _integral_phi_PI;
        double _integral_psi_PI;

        double _theta_dot;
        double _phi_dot;
        double _psi_dot;

        double _K_P_theta;
        double _K_I_theta;
        double _K_D_theta;
        double _K_P_phi;
        double _K_I_phi;
        double _K_D_phi;
        double _K_P_psi;
        double _K_I_psi;
        double _K_D_psi;

        double _RCAH_P_psi;
        double _RCAH_I_psi;
        double _RCAH_P_phi;
        double _RCAH_I_phi;
        double _RCAH_P_theta;
        double _RCAH_I_theta;

    public:

        double _dt;
        ros::Time _now;
        ros::Time _last;
        ros::Duration _ros_dt;
        phx_uart_msp_bridge::Motor _thrusts;

        trajectory_controller(ros::NodeHandle nh); // constructor

        // sets current_path
        void path_callback(const nav_msgs::Path::ConstPtr& msg);

        void set_current_pose(const geometry_msgs::Pose::ConstPtr& msg);
        void set_current_goal(const geometry_msgs::Pose::ConstPtr& msg);
        void calc_delta_x_dot();
        void calc_controller_error();
        void transform_quaternion();
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void set_thrusts();

        void do_controlling(ros::Publisher);
        double convert_thrust(double newton);
};

#endif
