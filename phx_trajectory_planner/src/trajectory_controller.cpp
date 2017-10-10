// !!! This is not a trajectory controller yet !!! Only attitude stabilization based on Simulink / Simmechanics model

/* open points:
    1.) Reglerparameter fine tunen mit Simulink
    2.) Masseeigenschaften Simulink Modell ueberpruefen
    3.) limit integral --> prevent integral wind-up sinnvoll ueberlegen!!
    4.) passt winkel trafo von quaternionen zu Eulerwinkel?? Drehreihenfolge XYZ??
    5.) passen Eulerwinkel und Drehraten prinzipiell? --> ueberpreufen (z.B. Koordinatentrafo bei Drehraten aber nicht Winkel gemacht...)
    6.) Drehraten Einheit passt?
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
  /*_k = 0; not needed in new implementation
  _b = 0; 
  _Ixx = 0;
  _Iyy = 0;
  _Izz = 0;
  _L = 0;*/ // distance from cog to any of the propellers
	
  _phi_cmd = 0; // Kommandogroessen in rad!
  _theta_cmd = 0;
  //_psi_cmd = 0; psi erst mal nur Rate zu 0 regeln wg. Problemen bei erstem Test
  _p_cmd = 0; // [rad/s]
  _q_cmd = 0;
  _r_cmd = 0;
	
  _u_p = 0; // Regleroutputs (Drehraten)
  _u_q = 0;
  _u_r = 0;
	
  _e_p = 0; // Reglerinputs Drehraten
  _e_q = 0;
  _e_r = 0;
  _last_e_p = 0; // for integration
  _last_e_q = 0;
  _last_e_r = 0;	

  _theta = 0; // current states
  _phi = 0;
  _psi = 0;
  _p = 0;
  _q = 0;
  _r = 0;
	
  /*_integral_theta = 0; // noch keinen I Anteil in Winkel
  _integral_phi = 0;
  _integral_psi = 0;*/
  _integral_p = 0;
  _integral_q = 0;
  _integral_r = 0;
  
  //_limit_integral = 0.2;
  _limit_integral = _m*_g/6;

  _dt = 0;
	
  _K_P_phi = 0.56252;
  _K_P_theta = 1.59269;
  //_K_P_psi = 0;
  _K_P_p = 0.08877;
  _K_I_p = 0.25469;
  _K_P_q = 0.16285;
  _K_I_q = 1.3512405;
  _K_P_r = 0.18682;
  _K_I_r = 37.3649;
	
  /*_K_I_theta = 0; old implementation
  _K_P_theta = 0;
  _K_D_theta = 0;
  _K_I_phi = 0;
  _K_P_phi = 0;
  _K_D_phi = 0;
  _K_I_psi = 0;
  _K_P_psi = 0;
  _K_D_psi = 0;*/

  /*nh.getParam("/trajectory_controller/thrust_rpm_const_k", _k);
  nh.getParam("/trajectory_controller/torque_drag_const_b", _b);
  nh.getParam("/trajectory_controller/I_xx", _Ixx);
  nh.getParam("/trajectory_controller/I_yy", _Iyy);
  nh.getParam("/trajectory_controller/I_zz", _Izz);
  nh.getParam("/trajectory_controller/dist_cog_prop", _L);*/

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
  // ====================================================================
  // angenommen Drehraten kommen in [°/s]! --> Umrechnung in rad/s notwendig
  // Koordinatentrafo nicht auch für Eulerwinkel notwendig?!?!
  // ====================================================================	
	
  double x_imu = msg->angular_velocity.x;
  double y_imu = msg->angular_velocity.y;
  _r = msg->angular_velocity.z*pi/180; // !Vorzeichen!; in rad/s umrechnen

  _current.orientation = msg->orientation;
  transform_quaternion();
	
  // do coordinate frame trafo from imu_coosy --> coosy in paper
  double root2 = sqrt(1/2);

  _p = root2 * (x_imu + y_imu)*pi/180; // in rad/s umrechnen
  _q = root2 * (x_imu - y_imu)*pi/180;
}

//Transform the quaternions into rotations about the coordinate axes
void trajectory_controller::transform_quaternion()
{
    tf2::Quaternion q;
    tf2::fromMsg(_current.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(_phi, _theta, _psi); // in rad
}

// Calculates controller error as suggested in paper
void trajectory_controller::calc_controller_outputs()
{
  // Attitude
  double e_phi = _phi_cmd - _phi;
  double e_theta = _theta_cmd - _theta;
  //e_psi = _psi_cmd - _psi; erst mal rausgenommen wg. Problemen bei erstem Test
  // Regleroutputs outer loop	
  double u_phi = _K_P_phi * e_phi;
  double u_theta = _K_P_theta * e_theta;
  double u_psi = 0;

  // Rates
  _e_p = u_phi - _p; // Regler Inputs inner loop
  _e_q = u_theta - _q;
  _e_r = u_psi - _r;
	  
  if (abs(_integral_p) > _limit_integral)
  {
      _integral_p = _limit_integral;
  }
  else if(isnan(_integral_p)) // needed for fixing mistake when random NaN appeared
  {
      _integral_p = 0;	  
  }
  else
  {
      _integral_p += (_e_p - _last_e_p) * _dt;  
  }
	  
  if (abs(_integral_q) > _limit_integral)
  {
      _integral_q = _limit_integral;
  }
  else if(isnan(_integral_q)) // needed for fixing mistake when random NaN appeared
  {
      _integral_q = 0;	  
  }
  else
  {
      _integral_q += (_e_q - _last_e_q) * _dt;  
  }
	  
  if (abs(_integral_r) > _limit_integral)
  {
      _integral_r = _limit_integral;
  }
  else if(isnan(_integral_r)) // needed for fixing mistake when random NaN appeared
  {
      _integral_r = 0;	  
  }
  else
  {
      _integral_r += (_e_r - _last_e_r) * _dt;
  }
	  
  _u_p = _K_P_p * _e_p + _K_I_p * _integral_p;
  _u_q = _K_P_q * _e_q + _K_I_q * _integral_q;
  _u_r = _K_P_r * _e_r + _K_I_r * _integral_r;
  	  
  /*	old version  
  						macht verodern von allem hier wirklich Sinn? siehe oben
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

  //_e_psi = _K_D_psi * _psi_dot + _K_P_psi * _psi + _K_I_psi * _integral_psi;
  _e_psi = 0;
  _e_phi = _K_D_phi * _phi_dot + _K_P_phi * _phi + _K_I_phi * _integral_phi;
  _e_theta = _K_D_theta * _theta_dot + _K_P_theta * _theta + _K_I_theta * _integral_theta;
  */
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
  double cog_correction = 0.365; // voruebergehend - Anpassung an Schwerpunktslage
  thrustsNewton[0] = gravity_norm + cog_correction + _u_q - _u_r;
  thrustsNewton[1] = gravity_norm + cog_correction + _u_q + _u_r;
  thrustsNewton[2] = gravity_norm - _u_p - _u_r;
  thrustsNewton[3] = gravity_norm - cog_correction - _u_q + _u_r;
  thrustsNewton[4] = gravity_norm - cog_correction - _u_q - _u_r;
  thrustsNewton[5] = gravity_norm + _u_p + _u_r;
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

    _last_e_p = _e_p; // prepare for next iteration
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

    // TODO motormsg
    ros::Publisher MotorMsg = nh.advertise<phx_uart_msp_bridge::Motor>("/phx/fc/motor_set", 1);

    controller.do_controlling(MotorMsg);

    return 0;
}
