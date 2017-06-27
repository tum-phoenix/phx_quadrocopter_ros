// implements a pid_controller as modeled in the paper "Quadrocopter Dynamics, Simulation and Control"
// !!! This is not a trajectory controller yet !!! Only stability as in paper

#include "phx_trajectory_planner/trajectory_controller.h"

// constructor
trajectory_controller::trajectory_controller(ros::NodeHandle nh)
{
  // get Parameters specified in launchfile
  // all in SI units
  _m = 0;
  _g = 9.81; // m/s^2
  _k = 0;
  _b = 0;
  _Ixx = 0;
  _Iyy = 0;
  _Izz = 0;
  _L = 0; // distance from cog to any of the propellers
  _e_theta = 0;
  _e_phi = 0;
  _e_psi = 0;
  _theta = 0;
  _phi = 0;
  _psi = 0;
  _last_theta = 0;
  _last_phi = 0;
  _last_psi = 0;
  _theta_dot = 0;
  _phi_dot = 0;
  _psi_dot = 0;
  _dt = 0;

  nh.getParam("/trajectory_controller/mass", _m);
  nh.getParam("/trajectory_controller/thrust_rpm_const_k", _k);
  nh.getParam("/trajectory_controller/torque_drag_const_b", _b);
  nh.getParam("/trajectory_controller/I_xx", _Ixx);
  nh.getParam("/trajectory_controller/I_yy", _Iyy);
  nh.getParam("/trajectory_controller/I_zz", _Izz);
  nh.getParam("/trajectory_controller/dist_cog_prop", _L);

}

// callback function for path_sub (updates current path, current and goal)
void trajectory_controller::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
  _current_path.header = msg->header;
  _current_path.poses = msg->poses;
  _current = _current_path.poses[0].pose;
  _current_goal = _current_path.poses[1].pose;
}

/*void trajectory_controller::set_current_pose(const geometry_msgs::Pose::ConstPtr& msg)//FIXME: This could be called pose callback
{
  // save the old values
  last_theta = theta;
  last_phi = phi;
  last_psi = psi;
  // get the new ones from the msg
  current.position = msg->position;
  current.orientation = msg->orientation;
  // calculate euler angles
  transform_quaternion();
  // caluculate the controller error
  calc_controller_error();
}*/

/*
void trajectory_controller::set_current_goal(const geometry_msgs::Pose::ConstPtr& msg)
{
  _current_goal.position = msg->position;
  _current_goal.orientation = msg->orientation;
}
*/

// callback for imu_pose subscriber
void trajectory_controller::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double x_imu = msg->angular_velocity.x;
  double y_imu = msg->angular_velocity.y;
  _psi_dot = msg->angular_velocity.z;


  // do coordinate frame trafo from imu_coosy --> coosy in paper
  double root2 = sqrt(1/2);

  _phi_dot = root2 * (x_imu + y_imu);
  _theta_dot = root2 * (x_imu - y_imu);
}

// calculates controller error as suggested in paper
void trajectory_controller::calc_controller_error()
{
  _e_psi = _K_D * _psi_dot + _K_P * _psi + _K_I * (_psi - _last_psi) * _dt;
  _e_phi = _K_D * _phi_dot + _K_P * _phi + _K_I * (_phi - _last_phi) * _dt;
  _e_theta = _K_D * _theta_dot + _K_P * _theta + _K_I * (_theta - _last_theta) * _dt;
}

void trajectory_controller::transform_quaternion()
{
    tf2::Quaternion q;
    tf2::fromMsg(_current.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(_phi, _theta, _psi);
}

// calcs thrusts according to paper
void trajectory_controller::set_thrusts()
{
  double gravity_norm = _m * _g / ( 4*cos(_theta)*cos(_psi) );

  // Einzelschuebe in Newton
  double T1 = gravity_norm - ( 2*_b*_e_phi*_Ixx + _e_psi*_Izz*_k*_L )/( 4*_b*_L );
  double T2 = gravity_norm + ( _k*_e_psi*_Izz )/( 4*_b ) - ( _e_theta*_Iyy )/( 2*_L );
  double T3 = gravity_norm - ( -2*_b*_e_phi*_Ixx + _e_psi*_Izz*_k*_L )/( 4*_b*_L );
  double T4 = gravity_norm + ( _k*_e_psi*_Izz )/( 4*_b ) + ( _e_theta*_Iyy )/( 2*_L );

  // TODO in prozent umrechnen und über MotorMsg publishen

}

void trajectory_controller::do_one_iteration()
{
  _now = ros::Time::now(); // get_current_time

  if(_dt != -1)
  {
    _ros_dt = _now - _last;
    _dt = _ros_dt.toSec();
  }
  else
  {
    _dt = 0; // for calc_controller_error
  }

  transform_quaternion();
  calc_controller_error();

  set_thrusts();

  _last_theta = _theta; // prepare for next iteration
  _last_psi = _psi;
  _last_phi = _phi;

  _last = _now; // time
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle nh;

    trajectory_controller controller(nh); // init class

    // subscribe to class trajectory_controller controller's method set_path
    ros::Subscriber path_sub = nh.subscribe("/phx/path", 1, &trajectory_controller::path_callback, &controller);

    //ros::Subscriber init_sub = nh.subscribe("/phx/pose", 1, &trajectory_controller::set_current_pose, &controller);
    ros::Subscriber imu_pose = nh.subscribe("/phoenix/imu", 10, &trajectory_controller::imu_callback, &controller);

    // TODO motormsg
    //ros::Publisher MotorMsg = nh.advertise<>("/phoenix/cmd_motor", 10);

    // wie oft publishen imu_pose?

    ros::Rate loop_rate(50);

    controller._dt = -1; // um den ersten loop durchgang zu checken

    while(ros::ok())
    {
        controller.do_one_iteration();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
