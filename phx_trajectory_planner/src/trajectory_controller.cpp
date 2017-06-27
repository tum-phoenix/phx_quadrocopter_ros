// implements a pid_controller as modeled in the paper "Quadrocopter Dynamics, Simulation and Control"
// !!! This is not a trajectory controller yet !!! Only stability as in paper

#include "phx_trajectory_planner/trajectory_controller.h"

trajectory_controller::trajectory_controller(ros::NodeHandle nh)
{
  // get Parameters specified in launchfile
  // all in SI units


  m = 0;
  g = 9.81; // m/s^2 
  k = 0;
  b = 0;
  Ixx = 0;
  Iyy = 0;
  Izz = 0;
  L = 0; // distance from cog to any of the propellers
  e_theta = 0;
  e_phi = 0;
  e_psi = 0;
  theta = 0;
  phi = 0;
  psi = 0;
  last_theta = 0;
  last_phi = 0;
  last_psi = 0;
  theta_dot = 0;
  phi_dot = 0;
  psi_dot = 0;

  nh.getParam("/trajectory_controller/mass", m);
  nh.getParam("/trajectory_controller/thrust_rpm_const_k", k);
  nh.getParam("/trajectory_controller/torque_drag_const_b", b);
  nh.getParam("/trajectory_controller/I_xx", Ixx);
  nh.getParam("/trajectory_controller/I_yy", Iyy);
  nh.getParam("/trajectory_controller/I_zz", Izz);
  nh.getParam("/trajectory_controller/dist_cog_prop", L);

}

void trajectory_controller::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
  current_path.header = msg->header;
  current_path.poses = msg->poses;
  current = current_path.poses[0].pose;
  current_goal = current_path.poses[1].pose;
}

/*
void trajectory_controller::set_current_pose(const geometry_msgs::Pose::ConstPtr& msg)
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
}
*/

void trajectory_controller::set_current_goal(const geometry_msgs::Pose::ConstPtr& msg)
{
  current_goal.position = msg->position;
  current_goal.orientation = msg->orientation;
}

void trajectory_controller::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  float x_imu = msg->angular_velocity.x;
  float y_imu = msg->angular_velocity.y;
  psi_dot = msg->angular_velocity.z;

  float root2 = sqrt(1/2);

  phi_dot = root2 * (x_imu + y_imu);
  theta_dot = root2 * (x_imu - y_imu);
}

void trajectory_controller::calc_controller_error()
{
  e_psi = K_D * psi_dot + K_P * psi + K_I * (psi - last_psi) * dt;
  e_phi = K_D * phi_dot + K_P * phi + K_I * (phi - last_phi) * dt;
  e_theta = K_D * theta_dot + K_P * theta + K_I * (theta - last_theta) * dt;
}

void trajectory_controller::transform_quaternion()
{
    tf2::Quaternion q;
    tf2::fromMsg(current.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(phi, theta, psi);
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
    //ros::Publisher MotorMsg = nh.advertise<>("/phoenix/cmd_motor", 10);

    ros::Rate loop_rate(50);


    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
