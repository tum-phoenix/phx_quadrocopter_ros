// !!! attitude stabilization and altitude hold controller based on Simulink / Simmechanics model

// ##########################################
// PID TUNING @ NI.com
// 
// The gains of a PID controller can be obtained by trial and error method.
// Once an engineer understands the significance of each gain parameter,
// this method becomes relatively easy. In this method, the I and D terms are set to zero first and
// the proportional gain is increased until the output of the loop oscillates. As one increases 
// the proportional gain, the system becomes faster, but care must be taken not make the system unstable.
// Once P has been set to obtain a desired fast response, the integral term is increased to stop the oscillations.
// The integral term reduces the steady state error, but increases overshoot. Some amount of overshoot
// is always necessary for a fast system so that it could respond to changes immediately. The integral term
// is tweaked to achieve a minimal steady state error. Once the P and I have been set to get the desired
// fast control system with minimal steady state error, the derivative term is increased until the loop
// is acceptably quick to its set point. Increasing derivative term decreases overshoot and yields higher
// gain with stability but would cause the system to be highly sensitive to noise. 
// Often times, engineers need to tradeoff one characteristic of a control system for another to better
// meet their requirements.
//
// ##########################################

/* open points:
    1.) filter turnrates, Offset? (--> check by integration and comparison with angles)
    2.) Position hold, use Imu Acceleration
    3.) Trajectory Controller
    .
    .
*/

// ##########################################
// throttle dT mode in 1% steps
// starting from 0: m*g/6 also commented out
// --> Altitude Hold is commented out
// ##########################################

#include "phx_trajectory_planner/trajectory_controller.h"

// constructor
trajectory_controller::trajectory_controller(ros::NodeHandle nh)
{
  // get Parameters specified in launchfile
  // all in SI units
  _m = 0;
  nh.getParam("/trajectory_controller/mass", _m);
	
  _phi_cmd = 0; // Kommandogroessen in rad!
  _theta_cmd = 0;
  //_psi_cmd = 0; psi: only command rate to 0 for now due problems in first test
  _altitude_cmd = 0.2;
  _r_cmd = 0;
	
  _u_p = 0; // controller outputs (PI turnrates)
  _u_q = 0;
  _u_r = 0;
  _dT = 0; // additional throttle
	
  _e_phi = 0; // controller inputs (PID angles)
  _e_theta = 0;
  //_e_psi = 0;
  _e_p = 0; // controller inputs (PI turnrates)
  _e_q = 0;
  _e_r = 0;
  _last_e_phi = 0;
  _last_e_theta = 0;
  //_last_e_psi = 0;
  _last_e_p = 0;
  _last_e_q = 0;
  _last_e_r = 0;	
  _last_diff_e_phi = 0;
	_last_diff_e_theta = 0;

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

  _dt = 0;
	
  // numbers are defined in header (+ given by Simulink for easier tuning)
  _K_P_phi = K_P_phi; // PID roll
  _K_I_phi = K_I_phi;
  _K_D_phi = K_D_phi;
  _K_P_theta = K_P_theta; // PID ritch
  _K_I_theta = K_I_theta;
  _K_D_theta = K_D_theta;
  //_K_P_psi = 0;
  _K_P_p = K_P_p; // PI rollrate
  _K_I_p = K_I_p;
  _K_P_q = K_P_q; // PI pitchrate
  _K_I_q = K_I_q;
  _K_P_r = K_P_r; // PI yawrate
  _K_I_r = K_I_r;
	
	// altitude hold
  _K_P_alt = K_P_alt;
  _K_I_alt = K_I_alt;
  _K_D_alt = K_D_alt;
  _K_N_alt = K_N_alt; // filter coefficient

	_altitude = 0;
	_last_altitude = 0;
	_last_alt_t = -1; // time, -1 for initialization (first if in altitude_callback)
  _wg = 4; // cutoff frequency for lowpass filter
	_integral_alt = 0;
	_e_alt = 0;
	_last_e_alt = 0;
  _limit_integral_altitude = 10; // determined by simulation, response to 1 m command
  _last_diff_e_alt = 0;
	
	_flg_I_control = 0;
	_flg_mtr_stop = 0;

  _lastthrustsNewton[0] = 0;
  _lastthrustsNewton[1] = 0;
  _lastthrustsNewton[2] = 0;
  _lastthrustsNewton[3] = 0;
  _lastthrustsNewton[4] = 0;
  _lastthrustsNewton[5] = 0;

}

// callback for gains subscriber (published by Simulink)
void trajectory_controller::gains_callback(const phx_trajectory_planner::Gains::ConstPtr& msg)
{
  //First do safety check so gains aren't changed mid flight
  //TODO: Find valid thresholds
  if ((_altitude < 0) && (_p == 0) && (_q == 0) && (_r == 0))
  {
    _K_P_phi = msg->K_Pphi; // PID Roll
    _K_I_phi = msg->K_Iphi;
    _K_D_phi = msg->K_Dphi;
    _K_P_theta = msg->K_Ptheta; // PID Pitch
    _K_I_theta = msg->K_Itheta;
    _K_D_theta = msg->K_Dtheta;
    _K_P_p = msg->K_Pp; // PI rollrate
    _K_I_p = msg->K_Ip;
    _K_P_q = msg->K_Pq; // PI pitchrate
    _K_I_q = msg->K_Iq;
    _K_P_r = msg->K_Pr; // PI yawrate
    _K_I_r = msg->K_Ir;

    _K_P_alt = msg->K_Palt;
    _K_I_alt = msg->K_Ialt;
    _K_D_alt = msg->K_Dalt;
    _K_N_alt = msg->K_Nalt; // filter coefficient
  }
}

// callback function for path_sub (updates current path, current and goal)
void trajectory_controller::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
  _current_path.header = msg->header;
  _current_path.poses = msg->poses;
  _current = _current_path.poses[0].pose;
  _current_goal = _current_path.poses[1].pose;
}

void trajectory_controller::ssh_rc_callback(const phx_uart_msp_bridge::RemoteControl::ConstPtr& msg)
{
  _phi_cmd = (msg->roll)*(1.0*M_PI/(2.0*180)); // in [rad] umrechnen !!
  _theta_cmd = (msg->pitch)*(1.0*M_PI/(2.0*180));
  _r_cmd = (msg->aux3)*(1.0*M_PI/(2.0*180));
  
  //std::cout << "roll_cmd: " << _phi_cmd << " [rad] " << "pitch_cmd: " << _theta_cmd << " [rad] " << "Throttle_delta: " << std::endl;
  //_psi_cmd = msg->yaw;

  // altitude cmd
  //_altitude_cmd = msg->aux1*1.0/10;
	
	_flg_mtr_stop = msg->aux2;

  // throttle cmd --> unten altitude hold regler auskommentieren!
  _dT = msg->aux1*1.0*MAXTNEWTON/100.0; // in 1% Schritten
}

void trajectory_controller::rc_callback(const phx_uart_msp_bridge::RemoteControl::ConstPtr& msg)
{
  if((msg->aux4 > 1600) && (_flg_I_control == 0) && (_dT > 2.0*MAXTNEWTON/100.0))
  {
  	_flg_I_control = 1;
  	std::cout << "I Control on" << std::endl;
  }
  else if(((msg->aux4 < 1600) || (_dT <= 2.0*MAXTNEWTON/100.0)) && (_flg_I_control == 1))
  {
  	_flg_I_control = 0;
  	std::cout << "I Control off" << std::endl;
  }
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
  // getting turnrates in [°/s]! --> transformation to rad/s necessary
  // no coordinate transformation necessary
  // in standard body fixed system
  // ====================================================================	
          // conversion s. UART MSP BRIDGE SRC
  _p = (msg->angular_velocity.x)*(M_PI/180.0)*(2000.0/8192.0);
  _q = -(msg->angular_velocity.y)*(M_PI/180.0)*(2000.0/8192.0); // MINUS determined in experiment
  _r = -(msg->angular_velocity.z)*(M_PI/180.0)*(2000.0/8192.0); // MINUS determined in experiment
}

//Callback for attitude subscriber (getting current euler angles)
void trajectory_controller::attitude_callback(const phx_uart_msp_bridge::Attitude::ConstPtr& msg)
{
  // Euler angles
  _phi = (double) msg->roll*M_PI/180;
  _theta = (double) -msg->pitch*M_PI/180; // MINUS determined in experiment
  _psi = (double) msg->yaw*M_PI/180;
}

// lowpass filter for Lidar Altitude
void trajectory_controller::altitude_callback(const phx_uart_msp_bridge::Altitude::ConstPtr& msg)
{
	if(_last_alt_t == -1)
	{
		_altitude = msg->estimated_altitude;
		_last_alt_t = msg->header.stamp.nsec*1.0/(1000000000) + msg->header.stamp.sec;
	}
	else
	{
		_last_altitude = _altitude;
		double alt_raw = msg->estimated_altitude;
		double t = msg->header.stamp.nsec*1.0/(1000000000) + msg->header.stamp.sec;
		double dt = t - _last_alt_t;
		
    // lowpass filter
		_altitude = (alt_raw*_wg*dt + _last_altitude)/(1 +_wg*dt);
		_last_alt_t = t;
	}
}

double trajectory_controller::integrate(double last_integral, double error, double last_error, double limit_integral)
{
  if(isnan(last_integral)) // needed for fixing mistake when random NaN appeared
  {
    last_integral = 0;
  }
	
  double integral = last_integral + (error + last_error) * _dt/2; // discrete trapezoidal integration
	
  if(integral > limit_integral)
  {
    integral = limit_integral;
  }
  else if(integral < -limit_integral)
  {
    integral = -limit_integral;
  }
	
  if(isnan(integral)) // needed for fixing mistake when random NaN appeared
  {
    integral = 0;
  }
	
  return integral;
}

double trajectory_controller::constrain(double value, double lower_limit, double upper_limit)
{
	if(value > upper_limit)
  {
    return upper_limit;
  }
  else if(value < lower_limit)
  {
    return lower_limit;
  }
  else
  {
  	return value;
  }
}

// Calculates controller error as suggested in paper
void trajectory_controller::calc_controller_outputs(ros::Publisher RateCmdMsg)
{
  // Attitude Controller
  _e_phi = _phi_cmd - _phi;
  _e_theta = _theta_cmd - _theta;
  //e_psi = _psi_cmd - _psi; commented out due to problems in first test

  // I-part
	if(_flg_I_control == 1)
	{
    _integral_phi = integrate(_integral_phi, _e_phi, _last_e_phi, 0.2); // limit determined by simulation
    _integral_theta = integrate(_integral_theta, _e_theta, _last_e_theta, 0.2); // limit determined by simulation
  }
  
  // D-part
  double diff_e_phi = 0;
  double diff_e_theta = 0;
  if(_dt != 0)
  {
    diff_e_phi = (_e_phi - _last_e_phi)/_dt; // differentiate
    diff_e_theta = (_e_theta - _last_e_theta)/_dt;
    
    // lowpass filter
    diff_e_phi = (diff_e_phi*K_N_phi*_dt + _last_diff_e_phi)/(1 +K_N_phi*_dt);
    diff_e_theta = (diff_e_theta*K_N_theta*_dt + _last_diff_e_theta)/(1 +K_N_theta*_dt);
    
		_last_diff_e_phi = diff_e_phi;
  	_last_diff_e_theta = diff_e_theta;
  }
	
  // controller outputs outer loop	= Attitude Controller
  double u_phi = _K_P_phi * _e_phi + _integral_phi * _K_I_phi + diff_e_phi * _K_D_phi; // p_cmd
  double u_theta = _K_P_theta * _e_theta + _integral_theta * _K_I_theta + diff_e_theta * _K_D_theta; // q_cmd
  double u_psi = 0; // r_cmd
	
	//u_phi = constrain(u_phi, -MAX_CMD_RATE, MAX_CMD_RATE);
	//u_theta = constrain(u_theta, -MAX_CMD_RATE, MAX_CMD_RATE); 

  // send message for analyzing controller
  _ratecmd.header.frame_id = "";
  _ratecmd.header.stamp = ros::Time::now();
  _ratecmd.roll = u_phi;
  _ratecmd.pitch = u_theta;
  _ratecmd.yaw = u_psi;
  RateCmdMsg.publish(_ratecmd);

  // Rate Controller
  //_e_p = u_phi - _p; // Regler Inputs inner loop  
  //_e_q = u_theta - _q;
  _e_r = _r_cmd - _r;

	if(_flg_I_control == 1)
	{
    //_integral_p = integrate(_integral_p, _e_p, _last_e_p, 0.2); // limit determined by simulation
		
    //_integral_q = integrate(_integral_q, _e_q, _last_e_q, 0.05); // limit determined by simulation
		
    _integral_r = integrate(_integral_r, _e_r, _last_e_r, 5); // limit determined by simulation
	}
	  
  //_u_p = _K_P_p * _e_p + _integral_p * _K_I_p;
  _u_p = u_phi;
  
  //_u_q = _K_P_q * _e_q + _integral_q * _K_I_q;
  _u_q = u_theta;
  
  _u_r = _K_P_r * _e_r + _integral_r * _K_I_r;	
	
	// Altitude
	/*
	_e_alt = _altitude_cmd - _altitude;
	
	if(_flg_I_control == 1)
	{
		_integral_alt = integrate(_integral_alt, _e_alt, _last_e_alt, _limit_integral_altitude);
	}
	
	double diff_e_alt = 0;
  if(_dt != 0)
  {
    diff_e_alt = _K_D_alt*_K_N_alt*(_e_alt - _last_e_alt) + (1 - _K_N_alt*_dt)*_last_diff_e_alt;
		_last_diff_e_alt = diff_e_alt;
  }
	
  _dT = _K_P_alt*_e_alt + _K_I_alt*_integral_alt + diff_e_alt;
  */
}

// converts thrust to throttle command PWM (1000 .... 2000)
int trajectory_controller::convert_thrust(double newton)
{
  double gramm = 1000*newton/G;
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

  // see prop.schubkennfeld
  // linear interpolation
  // if controller output is to high, thrust is set to 100
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
  val = 1000 + 10 * val; // convert to PWM for motor
  return (int)val;
}

// calcs thrusts according to paper
void trajectory_controller::set_thrusts()
{
  //double gravity_norm = _m * G / ( 6*cos(_theta)*cos(_phi) );
  //double gravity_norm = 0; //commented out due to first test

  // thrusts per rotor in Newton
  double thrustsNewton[6] = {0};
	
  // Control Matrix see Papers / Simulink Models
  // without gravity_norm
	// Hex Clean Flight Reihenfolge
  thrustsNewton[0] = _dT - 0.5*_u_p - _u_q + _u_r;
  thrustsNewton[1] = _dT - 0.5*_u_p + _u_q + _u_r;
  thrustsNewton[2] = _dT + 0.5*_u_p - _u_q - _u_r;
  thrustsNewton[3] = _dT + 0.5*_u_p + _u_q - _u_r;
  thrustsNewton[4] = _dT - _u_p - _u_r;
  thrustsNewton[5] = _dT + _u_p + _u_r;

  // convert to percent
  _thrusts.header.frame_id = "";
  _thrusts.header.stamp = ros::Time::now();

  //Constrain the rate of change of Thrust
  //14.9112 ist max. Thrust Newton
  for (int i = 0; i<6; i++){
    if ((thrustsNewton[i]-_lastthrustsNewton[i]) > (MAXTNEWTON / 50)) {
      thrustsNewton[i] = _lastthrustsNewton[i] + MAXTNEWTON / 50;
    }
    if ((thrustsNewton[i]-_lastthrustsNewton[i]) < (- MAXTNEWTON / 50)) {
      thrustsNewton[i] = _lastthrustsNewton[i] - MAXTNEWTON / 50;
    }
  }

	if(_flg_mtr_stop)
	{
		_thrusts.motor0 = MINCMDTHROTTLE; // min command
  	_thrusts.motor1 = MINCMDTHROTTLE;
  	_thrusts.motor2 = MINCMDTHROTTLE;
  	_thrusts.motor3 = MINCMDTHROTTLE;
  	_thrusts.motor4 = MINCMDTHROTTLE;
  	_thrusts.motor5 = MINCMDTHROTTLE;	
	}
	else
	{
		_thrusts.motor0 = convert_thrust(thrustsNewton[0]);
		_thrusts.motor1 = convert_thrust(thrustsNewton[1]);
		_thrusts.motor2 = convert_thrust(thrustsNewton[2]);
		_thrusts.motor3 = convert_thrust(thrustsNewton[3]);
		_thrusts.motor4 = convert_thrust(thrustsNewton[4]);
		_thrusts.motor5 = convert_thrust(thrustsNewton[5]);
	}

  for (int i = 0; i<6; i++){
    _lastthrustsNewton[i]=thrustsNewton[i];
  }
  //debug
    /*ROS_DEBUG("motor0 %d \n", _thrusts.motor0);
    ROS_DEBUG("motor1 %d \n", _thrusts.motor1);
    ROS_DEBUG("motor2 %d \n", _thrusts.motor2);
    ROS_DEBUG("motor3 %d \n", _thrusts.motor3);*/
}

void trajectory_controller::do_controlling(ros::Publisher MotorMsg, ros::Publisher RateCmdMsg)
{
  _dt = -1; //For the first loop
  ros::Rate loop_rate(100);  //Calculation Rate


  while(ros::ok())
  {
    _now = ros::Time::now(); // get_current_time

    if(_dt != -1)
    {
      _ros_dt = _now - _last;
      _dt = _ros_dt.toSec();
      if((_dt < 0.005) || (_dt > 0.015))
      {
      	std::cout << "Warning: Controller Loop Rate 100 Hz not achieved - actual Rate: " << 1/_dt << std::endl;
      }
    }
    else
    {
      _dt = 0; // for calc_controller_error
    }

    //transform_quaternion(); should only be executed when new orientation arives? --> now called in imu_callback
    //calc_delta_x_dot();
    calc_controller_outputs(RateCmdMsg);

    set_thrusts();

    // prepare for next iteration
    _last_e_phi = _e_phi;
    _last_e_theta = _e_theta;
    _last_e_p = _e_p;
    _last_e_q = _e_q;
    _last_e_r = _e_r;
		_last_e_alt = _e_alt;

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
	
		ros::Subscriber altitude = nh.subscribe("/phx/altitude", 1, &trajectory_controller::altitude_callback, &controller);
	
    // ssh remote control for trimming
    ros::Subscriber ssh_rc = nh.subscribe("/phx/ssh_rc", 1, &trajectory_controller::ssh_rc_callback, &controller);
    
    // remote control callback for enabling Controller and I-Control
    ros::Subscriber rc = nh.subscribe("/phx/fc/rc", 1, &trajectory_controller::rc_callback, &controller);

    // subscribe to gains published via Simulink to make tuning easier
    ros::Subscriber gains = nh.subscribe("phx/gains", 1, &trajectory_controller::gains_callback, &controller);

    // motorcmds
    ros::Publisher MotorMsg = nh.advertise<phx_uart_msp_bridge::Motor>("/phx/fc/motor_set", 1);

    // turnrates commanded by controller
    ros::Publisher RateCmdMsg = nh.advertise<phx_uart_msp_bridge::Attitude>("/phx/trajectory_controller", 1);

    controller.do_controlling(MotorMsg, RateCmdMsg);

    return 0;
}
