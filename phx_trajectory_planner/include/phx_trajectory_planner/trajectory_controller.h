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
#include "phx_uart_msp_bridge/Altitude.h"
#include "phx_uart_msp_bridge/RemoteControl.h"
//#include "roslib.h"

#define   G               9.81  //m/s² - gravity
#define   MAXTNEWTON      14.9112 //N - max Thrust of the motors
#define   MAX_CMD_RATE    300.0*M_PI/180 // 100 °/s
#define   MINCMDTHROTTLE  1000

// Reglerparameter
#define   K_P_phi			2.264305; // PID Roll
#define   K_I_phi			0.845257;
#define   K_D_phi			0.502417;
#define   K_P_theta		3.55258859; // PID Pitch
#define   K_I_theta		2.2899592; // falls test wieder nicht funktioniert, hier 0 reinschreiben!
#define   K_D_theta		0.6699618;
//#define   K_P_psi		0
#define   K_P_p				0.340461; // PI rollrate
#define   K_I_p				0.02371677;
#define   K_P_q				0.978288; // PI pitchrate
#define   K_I_q				0.5188156;
#define   K_P_r				0.18682464; // PI yawrate
#define   K_I_r				37.3649283;
	
	// altitude hold
#define   K_P_alt			0.02071;
#define   K_I_alt			0.00069123;
#define   K_D_alt			0.15357;
#define   K_N_alt			37.20857; // filter coefficient


class trajectory_controller
{

    private:
        nav_msgs::Path _current_path; // current path to follow
        geometry_msgs::Pose _current_goal; // current goal for controller
        geometry_msgs::Pose _current;

        double _m; // mass

        double _phi_cmd; // Kommandagroessen in rad
        double _theta_cmd;
        //double _psi_cmd; psi erst mal nur Rate zu 0 regeln wg. Problemen bei erstem Test
        double _p_cmd; // Rates
        double _q_cmd;
        double _r_cmd;
        double _altitude_cmd;
        double _dT; // throttle delta
    
        double _phi; // current states
        double _theta;
        double _psi;
        double _p; // Rates
        double _q;
        double _r;
    
        double _altitude;
        double _last_altitude;
        double _last_alt_t;
        double _wg;
        double _integral_alt;
        double _e_alt;
        double _last_e_alt;
        double _limit_integral_altitude;
        double _last_diff_e_alt;

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
        double _K_P_alt;
        double _K_I_alt;
        double _K_D_alt;
        double _K_N_alt;

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
        double _limit_integral_attitude;
    
        double _u_p; // Rate Controller Outputs
        double _u_q;
        double _u_r;

        int _flg_mtr_stop;
        int _flg_I_control;

        double _lastthrustsNewton[6]; //Needed for constraining the change in thrust
  
    public:

        double _dt;
        ros::Time _now;
        ros::Time _last;
        ros::Duration _ros_dt;
        phx_uart_msp_bridge::Motor _thrusts;
        phx_uart_msp_bridge::Attitude _ratecmd;

        trajectory_controller(ros::NodeHandle nh); // constructor

        // sets current_path
        void path_callback(const nav_msgs::Path::ConstPtr& msg);

        void set_current_pose(const geometry_msgs::Pose::ConstPtr& msg);
        void set_current_goal(const geometry_msgs::Pose::ConstPtr& msg);
        //void calc_delta_x_dot();
        void calc_controller_outputs(ros::Publisher RateCmdMsg);
        //void transform_quaternion();
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        void set_thrusts();
        void rc_callback(const phx_uart_msp_bridge::RemoteControl::ConstPtr& msg);
        void altitude_callback(const phx_uart_msp_bridge::Altitude::ConstPtr& msg);

        void do_controlling(ros::Publisher MotorMsg, ros::Publisher RateCmdMsg);
        int convert_thrust(double newton);
        double integrate(double last_integral, double error, double last_error, double limit_integral);
        void attitude_callback(const phx_uart_msp_bridge::Attitude::ConstPtr& msg);
};

#endif
