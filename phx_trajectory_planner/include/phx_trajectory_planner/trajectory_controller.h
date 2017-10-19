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
#include "phx_uart_msp_bridge/Attitude.h"
#include "phx_uart_msp_bridge/RemoteControl.h"
//#include "roslib.h"

class trajectory_controller
{

    private:
        nav_msgs::Path _current_path; // current path to follow
        geometry_msgs::Pose _current_goal; // current goal for controller
        geometry_msgs::Pose _current;

        double _m; // mass
        /* not needed anymore in new implementation
        double _k; // thrust_rpm_const_k
        double _b; // torque_drag_const_b

        double _Ixx; // Massentraegheitsmomente
        double _Iyy;
        double _Izz;

        double _L; // distance from COG to any one of the propellers
        */
        double _g; // gravity, 9.81 m/s²

        double _phi_cmd; // Kommandagroessen in rad
        double _theta_cmd;
        //double _psi_cmd; psi erst mal nur Rate zu 0 regeln wg. Problemen bei erstem Test
        double _p_cmd; // Rates
        double _q_cmd;
        double _r_cmd;
        double _dT; // throttle delta
    
        double _phi; // current states
        double _theta;
        double _psi;
        double _p; // Rates
        double _q;
        double _r;

        double _K_P_phi;
        double _K_I_phi;
        double _K_D_phi;
        double _K_P_theta;
        double _K_I_theta;
        double _K_D_theta;
        //double _K_P_psi;
        double _K_P_p;
        double _K_I_p;
        double _K_P_q;
        double _K_I_q;
        double _K_P_r;
        double _K_I_r;

        double _e_phi; // Reglerinputs (PID Winkel)
        double _e_theta;
        //double _e_psi;
        double _e_p; // Rate Controller Inputs
        double _e_q;
        double _e_r;
        double _last_e_phi;
        double _last_e_theta;
        double _last_e_p; // for integration
        double _last_e_q;
        double _last_e_r;
        double _integral_phi;
        double _integral_theta;
        double _integral_p;
        double _integral_q;
        double _integral_r;
        double _limit_integral;
        double _max_cmd_rate;
    
        double _u_p; // Rate Controller Outputs
        double _u_q;
        double _u_r;


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
        //void calc_delta_x_dot();
        void calc_controller_outputs();
        //void transform_quaternion();
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void set_thrusts();
        void rc_callback(const phx_uart_msp_bridge::RemoteControl::ConstPtr& msg);

        void do_controlling(ros::Publisher);
        int convert_thrust(double newton);
        double integrate(double last_integral, double error, double last_error);
        void attitude_callback(const phx_uart_msp_bridge::Attitude::ConstPtr& msg);
};

#endif
