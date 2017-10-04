// implements a pid_controller as modeled in the paper "Quadrocopter Dynamics, Simulation and Control"
// !!! This is not a trajectory controller yet !!! Only attitude stabilization as in paper


/* open points:
    1.) 6 propeller kompatibel -- CHECK
    2.) reglerparameter (K_I, K_P, K_D) -- CHECK
    3.) motor msgs schicken -- CHECK
    4.) I anteil nur zuschalten wenn innerhalb gewisser genauigkeit -- CHECK
    5.) Simulation
    6.) Testen, ob es bisher soweit funktioniert
    .
    .
    .
    Unendlich.) trajektorienfolgeregler (bis jetzt nur lage)
*/

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
  _cmd_p = 0;
  _cmd_q = 0;
  _cmd_r = 0;
  _theta = 0;
  _phi = 0;
  _psi = 0;
  _last_theta = 0;
  _last_phi = 0;
  _last_psi = 0;
  _integral_theta = 0;
  _integral_phi = 0;
  _integral_psi = 0;
  _integral_theta_PI = 0;
  _integral_phi_PI = 0;
  _integral_psi_PI = 0;
  _limit_integral = 0.2;
  _theta_dot = 0;
  _phi_dot = 0;
  _psi_dot = 0;
  _dt = 0;
  _K_I_theta = 0;
  _K_P_theta = 0;
  _K_D_theta = 0;
  _K_I_phi = 0;
  _K_P_phi = 0;
  _K_D_phi = 0;
  _K_I_psi = 0;
  _K_P_psi = 0;
  _K_D_psi = 0;

  /*
  _RCAH_P_theta = 0;
  _RCAH_I_theta = 0;
  _RCAH_P_phi = 0;
  _RCAH_I_phi = 0;
  _RCAH_P_psi = 0;
  _RCAH_I_psi = 0;
  */

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
  _psi_dot = - msg->angular_velocity.z;

  _current.orientation = msg->orientation;
    
  // do coordinate frame trafo from imu_coosy --> coosy in paper
  double root2 = sqrt(1/2);

  _phi_dot = root2 * (x_imu + y_imu);
  _theta_dot = root2 * (x_imu - y_imu);
}

//Transform the quaternions into rotations about the coordinate axes
void trajectory_controller::transform_quaternion()
{
    tf2::Quaternion q;
    tf2::fromMsg(_current.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(_phi, _theta, _psi);
}


//Second controller to control the angles
/*void trajectory_controller::calc_delta_x_dot()
{
  if (abs(_integral_theta_PI) > 0.01 || abs(_integral_phi_PI) > 0.01 || abs(_integral_psi_PI) > 0.01)
  {
      _integral_theta_PI = 0;
      _integral_phi_PI = 0;
      _integral_psi_PI = 0;
  }
  else
  {
      _integral_theta_PI += (_theta - _last_theta) * _dt;
      _integral_phi_PI += (_phi - _last_phi) * _dt;
      _integral_psi_PI += (_psi - _last_psi) * _dt;
  }

  //Needed to fix mistake where one or two NaNs appear
  if (isnan(_integral_theta_PI) || isnan(_integral_phi_PI) || isnan(_integral_psi_PI))
  {
      _integral_theta_PI = 0;
      _integral_phi_PI = 0;
      _integral_psi_PI = 0;
  }

  _cmd_r = _RCAH_P_psi * (_psi - _last_psi) + _RCAH_I_psi * _integral_psi_PI;
  _cmd_p = _RCAH_P_phi * (_phi - _last_phi) + _RCAH_I_phi * _integral_phi_PI;
  _cmd_q = _RCAH_P_theta * (_theta - _last_theta) + _RCAH_I_theta * _integral_theta_PI;

  _phi_dot -= _cmd_p;
  _theta_dot -= _cmd_q;
  _psi_dot -= _cmd_r;
}*/

// Calculates controller error as suggested in paper
void trajectory_controller::calc_controller_error()
{
  if (abs(_integral_theta) > _limit_integral || abs(_integral_phi) > _limit_integral || abs(_integral_psi) > _limit_integral)
  {
      _integral_theta = _limit_integral;
      _integral_phi = _limit_integral;
      _integral_psi = _limit_integral;
  }
  else
  {
      _integral_theta += (_theta - _last_theta) * _dt;
      _integral_phi += (_phi - _last_phi) * _dt;
      _integral_psi += (_psi - _last_psi) * _dt;
  }

  //Needed to fix mistake where one or two NaNs appear
  if (isnan(_integral_theta) || isnan(_integral_phi) || isnan(_integral_psi))
  {
      _integral_theta = 0;
      _integral_phi = 0;
      _integral_psi = 0;
  }

  _e_psi = _K_D_psi * _psi_dot + _K_P_psi * _psi + _K_I_psi * _integral_psi;
  _e_phi = _K_D_phi * _phi_dot + _K_P_phi * _phi + _K_I_phi * _integral_phi;
  _e_theta = _K_D_theta * _theta_dot + _K_P_theta * _theta + _K_I_theta * _integral_theta;
}

// converts thrust to throttle command in %
int trajectory_controller::convert_thrust(double newton)
{
  //double gramm = 1000*newton/_g;
  double gramm = 100;

  double val = 0;
  double a = 0;
  double b = 0;
  double c = 0;
  double d = 0;
	
  if(gramm < 0)
  {
	  gramm = 0;
  }

  // vgl. prop.schubkennfeld
  // linear interpolation
  // Wenn Regler zu hohen Schub gibt, wird 100 eingestellt
  if(gramm < 600)
  {
    a = 0;
    b = 600;
    c = 0;
    d = 50;
  }
  else if(gramm < 930)
  {
    a = 600;
    b = 930;
    c = 50;
    d = 65;
  }
  else if(gramm < 1150)
  {
    a = 930;
    b = 1150;
    c = 65;
    d = 75;
  }
  else if(gramm < 1350)
  {
    a = 1150;
    b = 1350;
    c = 75;
    d = 85;
  }
  else if(gramm < 1520)
  {
    a = 1350;
    b = 1520;
    c = 85;
    d = 100;
  }
  else
  {
    return 2000;
  }

  val = c + (d-c)*(gramm-a)/(b-a);
  val = 1025 + 975 * val/100;
  return (int)val;
}

// calcs thrusts according to paper
void trajectory_controller::set_thrusts()
{
  double gravity_norm = _m * _g / ( 6*cos(_theta)*cos(_phi) );

  // Einzelschuebe in Newton
  double thrustsNewton[6] = {0};	
  thrustsNewton[0] = gravity_norm + _e_phi*_Ixx/(6*_L) + _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) - _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[1] = gravity_norm + _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) - _e_phi*_Ixx/(6*_L) + _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[2] = gravity_norm - _e_phi*_Ixx/(3*_L) - _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[3] = gravity_norm - _e_phi*_Ixx/(6*_L) - _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) + _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[4] = gravity_norm - _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) + _e_phi*_Ixx/(6*_L) - _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[5] = gravity_norm + _e_phi*_Ixx/(3*_L) + _e_psi*_k*_Izz/(6*_b);
    
  //ROS_DEBUG("motor0 %lf \n", thrustsNewton[0]);
  //ROS_DEBUG("motor1 %lf \n", thrustsNewton[1]);
  //ROS_DEBUG("motor2 %lf \n", thrustsNewton[2]);
  //ROS_DEBUG("motor3 %lf \n", thrustsNewton[3]);

  // in prozent umrechnen
  _thrusts.header.frame_id = "";
  _thrusts.header.stamp = ros::Time::now();
  //Reihenfolge durch uart_bridge festgelegt
  _thrusts.motor0 = convert_thrust(thrustsNewton[3]);
  _thrusts.motor1 = convert_thrust(thrustsNewton[1]);
  _thrusts.motor2 = convert_thrust(thrustsNewton[4]);
  _thrusts.motor3 = convert_thrust(thrustsNewton[0]);
  _thrusts.motor4 = convert_thrust(thrustsNewton[2]);
  _thrusts.motor5 = convert_thrust(thrustsNewton[5]);
    
  //debug
    ROS_DEBUG("motor0 %d \n", _thrusts.motor0);
    ROS_DEBUG("motor1 %d \n", _thrusts.motor1);
    ROS_DEBUG("motor2 %d \n", _thrusts.motor2);
    ROS_DEBUG("motor3 %d \n", _thrusts.motor3);

  // Debug
  //std::cout << "T1: " << _thrusts.motor0 << "  T2: " << _thrusts.motor1 << std::endl;
  //std::cout << "T3: " << _thrusts.motor3 << "  T4: " << _thrusts.motor4 << std::endl;
  //std::cout << "-------" << std::endl;
}

void trajectory_controller::do_controlling(ros::Publisher MotorMsg)
{
  //Parameters
  _K_P_theta = 37.086;
  _K_I_theta = 41.91;
  _K_D_theta = 10.65;
  _K_P_phi = 8.816;
  _K_I_phi = 5.032;
  _K_D_phi = 5.14;
  _K_P_psi = 25.88;
  _K_I_psi = 23.926;
  _K_D_psi = 8.94;

/*
  _RCAH_P_theta = 4.174;
  _RCAH_I_theta = 12.893;
  _RCAH_P_phi = 5.496;
  _RCAH_I_phi = 11.761;
  _RCAH_P_psi = 3.75;
  _RCAH_I_psi = 4.942;
*/

  _dt = -1; //For the first loop
  ros::Rate loop_rate(50);  //Calculation Rate

  while(ros::ok())
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
    //calc_delta_x_dot();
    calc_controller_error();

    set_thrusts();

    _last_theta = _theta; // prepare for next iteration
    _last_psi = _psi;
    _last_phi = _phi;

    _last = _now; // time

    MotorMsg.publish(_thrusts);

    ros::spinOnce();

    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle nh;

    trajectory_controller controller(nh); // init class

    // subscribe to class trajectory_controller controller's method set_path
    //ros::Subscriber path_sub = nh.subscribe("/phx/path", 1, &trajectory_controller::path_callback, &controller);

    //ros::Subscriber init_sub = nh.subscribe("/phx/pose", 1, &trajectory_controller::set_current_pose, &controller);
    ros::Subscriber imu_pose = nh.subscribe("/phx/imu", 10, &trajectory_controller::imu_callback, &controller);

    // TODO motormsg
    ros::Publisher MotorMsg = nh.advertise<phx_uart_msp_bridge::Motor>("/phx/fc/motor_set", 10);

    // wie oft publishen imu_pose?

    controller.do_controlling(MotorMsg);

    return 0;
}
