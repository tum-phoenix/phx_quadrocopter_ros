// !!! This is not a trajectory controller yet !!! Only attitude stabilization based on Simulink / Simmechanics model

/* open points:
    1.) passen Vorzeichen von Eulerwinkel bzw Drehraten? v.a. psi? Drehraten filtern? (z.B. einfacher Tiefpass)
    2.) Regler für horizontale und vertikale Geschwindigkeit (Altitude Hold)
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
  nh.getParam("/trajectory_controller/mass", _m);
	
  _g = 9.81; // m/s^2
	
  _phi_cmd = 0; // Kommandogroessen in rad!
  _theta_cmd = 0;
  //_psi_cmd = 0; psi erst mal nur Rate zu 0 regeln wg. Problemen bei erstem Test
  _p_cmd = 0; // [rad/s]
  _q_cmd = 0;
  _r_cmd = 0;
  _dT = 0; // additional throttle
	
  _u_p = 0; // Regleroutputs (PI Drehraten)
  _u_q = 0;
  _u_r = 0;
  _e_phi = 0; // Reglerinputs (PID Winkel)
  _e_theta = 0;
  //_e_psi = 0;
  _e_p = 0; // Reglerinputs (PI Drehraten)
  _e_q = 0;
  _e_r = 0;
  _last_e_phi = 0;
  _last_e_theta = 0;
  //_last_e_psi = 0;
  _last_e_p = 0;
  _last_e_q = 0;
  _last_e_r = 0;	

  _theta = 0; // current states
  _phi = 0;
  _psi = 0;
  _p = 0;
  _q = 0;
  _r = 0;
	
  _integral_phi = 0;
  _integral_theta = 0;
  //_integral_psi = 0;
  _integral_p = 0;
  _integral_q = 0;
  _integral_r = 0;
  
  // 					max thrust
  _limit_integral_rates = (1.52 - _m/6)*_g;
  _max_cmd_rate = 100*M_PI/180; // 100 °/s

  _dt = 0;
	
  _K_P_phi = 2.264305; // PID Roll
  _K_I_phi = 0.845257;
  _K_D_phi = 0.502417;
  _K_P_theta = 2.9259899; // PID Pitch
  _K_I_theta = 1.9397181;
  _K_D_theta = 0.37002521;
  //_K_P_psi = 0;
  _K_P_p = 0.340461; // PI rollrate
  _K_I_p = 0.02371677;
  _K_P_q = 0.5492050; // PI pitchrate
  _K_I_q = 1.9120622;
  _K_P_r = 0.18682464; // PI yawrate
  _K_I_r = 37.3649283;

}

// callback function for path_sub (updates current path, current and goal)
void trajectory_controller::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
  _current_path.header = msg->header;
  _current_path.poses = msg->poses;
  _current = _current_path.poses[0].pose;
  _current_goal = _current_path.poses[1].pose;
}

void trajectory_controller::rc_callback(const phx_uart_msp_bridge::RemoteControl::ConstPtr& msg)
{
  //_phi_cmd = msg->roll;
  //_theta_cmd = msg->pitch;
  //_psi_cmd = msg->yaw;
  //_dT = msg->throttle;  
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
  // ====================================================================
  // Drehraten kommen in [°/s]! --> Umrechnung in rad/s notwendig
  // Koordinatentrafo nicht notwendig
  // ====================================================================	
					// Umrechnung s. UART MSP BRIDGE SRC
  _p = msg->angular_velocity.x*(M_PI/180)*(2000/8192);
  _q = msg->angular_velocity.y*(M_PI/180)*(2000/8192);
  _r = msg->angular_velocity.z*(M_PI/180)*(2000/8192);
	
  /*double x_imu = msg->angular_velocity.x;
  double y_imu = msg->angular_velocity.y;
  _r = msg->angular_velocity.z*M_PI/180; // !Vorzeichen!; in rad/s umrechnen

  _current.orientation = msg->orientation;
  transform_quaternion();
	
  // do coordinate frame trafo from imu_coosy --> coosy in paper
  double root2 = sqrt(1/2);

  _p = root2 * (x_imu + y_imu)*M_PI/180; // in rad/s umrechnen
  _q = root2 * (x_imu - y_imu)*M_PI/180;*/
}

// now getting euler angles from phx/fc/attitude
//Transform the quaternions into rotations about the coordinate axes
/*void trajectory_controller::transform_quaternion()
{
    tf2::Quaternion q;
    tf2::fromMsg(_current.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(_phi, _theta, _psi); // in rad
}*/

void trajectory_controller::attitude_callback(const phx_uart_msp_bridge::Attitude::ConstPtr& msg)
{
  _phi = (double) msg->roll*M_PI/180;
  _theta = (double) msg->pitch*M_PI/180;
  _psi = (double) msg->yaw*M_PI/180;
}

double trajectory_controller::integrate(double last_integral, double error, double last_error, double limit, double K_I)
{
  if(isnan(last_integral)) // needed for fixing mistake when random NaN appeared
  {
    last_integral = 0;
  }
	
  double integral = last_integral + (error + last_error) * K_I * _dt/2; // discrete trapezoidal integration
	
  if(integral > limit)
  {
    integral = limit;
  }
  else if(integral < -limit)
  {
    integral = -limit;
  }
	
  if(isnan(integral)) // needed for fixing mistake when random NaN appeared
  {
    integral = 0;
  }
	
  return integral;
}

// Calculates controller error as suggested in paper
void trajectory_controller::calc_controller_outputs()
{
  // Attitude
  _e_phi = _phi_cmd - _phi;
  _e_theta = _theta_cmd - _theta;
  //e_psi = _psi_cmd - _psi; erst mal rausgenommen wg. Problemen bei erstem Test

  _integral_phi = integrate(_integral_phi, _e_phi, _last_e_phi, _max_cmd_rate, _K_I_phi);
  _integral_theta = integrate(_integral_theta, _e_theta, _last_e_theta, _max_cmd_rate, _K_I_theta);
  
  double diff_e_phi = 0;
  double diff_e_theta = 0;
  if(_dt != 0)
  {
  	diff_e_phi = (_e_phi - _last_e_phi)/_dt; // differentiate
    diff_e_theta = (_e_theta - _last_e_theta)/_dt;
  }
	
  // Regleroutputs outer loop	
  double u_phi = _K_P_phi * _e_phi + _integral_phi + diff_e_phi * _K_D_phi; // p_cmd
  double u_theta = _K_P_theta * _e_theta + _integral_theta + diff_e_theta * _K_D_theta; // q_cmd
  double u_psi = 0; // r_cmd
	
  if(u_phi > _max_cmd_rate)
  {
    u_phi = _max_cmd_rate;
  }
  else if(u_phi < -_max_cmd_rate)
  {
    u_phi = -_max_cmd_rate;
  }
  if(u_theta > _max_cmd_rate)
  {
    u_theta = _max_cmd_rate;
  }
  else if(u_theta < -_max_cmd_rate)
  {
    u_theta = -_max_cmd_rate;
  }

  // Rates
  _e_p = u_phi - _p; // Regler Inputs inner loop
  _e_q = u_theta - _q;
  _e_r = u_psi - _r;

  _integral_p = integrate(_integral_p, _e_p, _last_e_p, _limit_integral_rates, _K_I_p);
  _integral_q = integrate(_integral_q, _e_q, _last_e_q, _limit_integral_rates, _K_I_q);
  _integral_r = integrate(_integral_r, _e_r, _last_e_r, _limit_integral_rates, _K_I_r);
	  
  _u_p = _K_P_p * _e_p + _integral_p;
  _u_q = _K_P_q * _e_q + _integral_q;
  _u_r = _K_P_r * _e_r + _integral_r;	
  /*	
  _u_p = _K_P_p * _e_p + _K_I_p * _integral_p;
  _u_q = _K_P_q * _e_q + _K_I_q * _integral_q;
  _u_r = _K_P_r * _e_r + _K_I_r * _integral_r;*/
}

// converts thrust to throttle command in %
int trajectory_controller::convert_thrust(double newton)
{
  double gramm = 1000*newton/_g;
  //double gramm = 100;

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
  val = 1025 + 975 * val/100; // convert to PWM for motor
  return (int)val;
}

// calcs thrusts according to paper
void trajectory_controller::set_thrusts()
{
  double gravity_norm = _m * _g / ( 6*cos(_theta)*cos(_phi) );

  // Einzelschuebe in Newton
  double thrustsNewton[6] = {0};
  // old implementation aus paper
  /*thrustsNewton[0] = gravity_norm + _e_phi*_Ixx/(6*_L) + _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) - _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[1] = gravity_norm + _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) - _e_phi*_Ixx/(6*_L) + _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[2] = gravity_norm - _e_phi*_Ixx/(3*_L) - _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[3] = gravity_norm - _e_phi*_Ixx/(6*_L) - _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) + _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[4] = gravity_norm - _e_theta*_Iyy/(4*sqrt(3)*_L*0.5) + _e_phi*_Ixx/(6*_L) - _e_psi*_k*_Izz/(6*_b);
  thrustsNewton[5] = gravity_norm + _e_phi*_Ixx/(3*_L) + _e_psi*_k*_Izz/(6*_b);*/
	
  // new implementation based on Simulink
  double cog_correction = 0.37; // voruebergehend - Anpassung an Schwerpunktslage
  thrustsNewton[0] = _dT + gravity_norm + cog_correction + _u_q - _u_r;
  thrustsNewton[1] = _dT + gravity_norm + cog_correction + _u_q + _u_r;
  thrustsNewton[2] = _dT + gravity_norm - _u_p - _u_r;
  thrustsNewton[3] = _dT + gravity_norm - cog_correction - _u_q + _u_r;
  thrustsNewton[4] = _dT + gravity_norm - cog_correction - _u_q - _u_r;
  thrustsNewton[5] = _dT + gravity_norm + _u_p + _u_r;
  /*thrustsNewton[0] = gravity_norm + cog_correction + _u_p + _u_q - _u_r;
  thrustsNewton[1] = gravity_norm + cog_correction - _u_p + _u_q + _u_r;
  thrustsNewton[2] = gravity_norm - 2*_u_p - _u_r;
  thrustsNewton[3] = gravity_norm - cog_correction - _u_p - _u_q + _u_r;
  thrustsNewton[4] = gravity_norm - cog_correction + _u_p - _u_q - _u_r;
  thrustsNewton[5] = gravity_norm + 2*_u_p + _u_r;*/
    
  //ROS_DEBUG("motor0 %lf \n", thrustsNewton[0]);
  //ROS_DEBUG("motor1 %lf \n", thrustsNewton[1]);
  //ROS_DEBUG("motor2 %lf \n", thrustsNewton[2]);
  //ROS_DEBUG("motor3 %lf \n", thrustsNewton[3]);

  // in prozent umrechnen
  _thrusts.header.frame_id = "";
  _thrusts.header.stamp = ros::Time::now();
  // Convert to Hex Clean Flight Reihenfolge
  _thrusts.motor0 = convert_thrust(thrustsNewton[3]);
  _thrusts.motor1 = convert_thrust(thrustsNewton[1]);
  _thrusts.motor2 = convert_thrust(thrustsNewton[4]);
  _thrusts.motor3 = convert_thrust(thrustsNewton[0]);
  _thrusts.motor4 = convert_thrust(thrustsNewton[2]);
  _thrusts.motor5 = convert_thrust(thrustsNewton[5]);
    
  //debug
    /*ROS_DEBUG("motor0 %d \n", _thrusts.motor0);
    ROS_DEBUG("motor1 %d \n", _thrusts.motor1);
    ROS_DEBUG("motor2 %d \n", _thrusts.motor2);
    ROS_DEBUG("motor3 %d \n", _thrusts.motor3);*/
}

void trajectory_controller::do_controlling(ros::Publisher MotorMsg)
{
  //Parameters
  // old version, now initialiazed in constructor
  /*_K_P_theta = 37.086;
  _K_I_theta = 41.91;
  _K_D_theta = 10.65;
  _K_P_phi = 8.816;
  _K_I_phi = 5.032;
  _K_D_phi = 5.14;
  _K_P_psi = 25.88;
  _K_I_psi = 23.926;
  _K_D_psi = 8.94;*/

  _dt = -1; //For the first loop
  ros::Rate loop_rate(100);  //Calculation Rate

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

    //transform_quaternion(); should only be executed when new orientation arives? --> now called in imu_callback
    //calc_delta_x_dot();
    calc_controller_outputs();

    set_thrusts();

    // prepare for next iteration
    _last_e_phi = _e_phi;
    _last_e_theta = _e_theta;
    _last_e_p = _e_p;
    _last_e_q = _e_q;
    _last_e_r = _e_r;

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
    ros::Subscriber imu_pose = nh.subscribe("/phx/imu", 1, &trajectory_controller::imu_callback, &controller);
	
    ros::Subscriber euler_angles = nh.subscribe("/phx/fc/attitude", 1, &trajectory_controller::attitude_callback, &controller);
	
	// phx/fc/rc oder phx/fc/rc_pilot?
	ros::Subscriber rc = nh.subscribe("/phx/fc/rc_pilot", 1, &trajectory_controller::rc_callback, &controller);

    // motorcmds
    ros::Publisher MotorMsg = nh.advertise<phx_uart_msp_bridge::Motor>("/phx/fc/motor_set", 1);

    controller.do_controlling(MotorMsg);

    return 0;
}
