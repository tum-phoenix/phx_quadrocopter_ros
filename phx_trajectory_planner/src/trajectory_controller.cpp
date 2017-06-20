// implements a pid_controller as modeled in the paper "Quadrocopter Dynamics, Simulation and Control"
// !!! This is not a trajectory controller yet !!! Only stability as in paper

#include "phx_trajectory_planner/trajectory_controller.h"

trajectory_controller::trajectory_controller(ros::NodeHandle nh)
{
  // get Parameters specified in launchfile
  // all in SI units
  this->m = 0;
  this->k = 0;
  this->b = 0;
  this->Ixx = 0;
  this->Iyy = 0;
  this->Izz = 0;
  this->L = 0; // distance from cog to any of the propellers
  this->e_theta = 0;
  this->e_phi = 0;
  this->e_psi = 0;
  this->theta = 0;
  this->phi = 0;
  this->psi = 0;
  this->last_theta = 0;
  this->last_phi = 0;
  this->last_psi = 0;
  this->theta_dot = 0;
  this->phi_dot = 0;
  this->psi_dot = 0;

  nh.getParam("/trajectory_controller/mass", this->m);
  nh.getParam("/trajectory_controller/thrust_rpm_const_k", this->k);
  nh.getParam("/trajectory_controller/torque_drag_const_b", this->b);
  nh.getParam("/trajectory_controller/I_xx", this->Ixx);
  nh.getParam("/trajectory_controller/I_yy", this->Iyy);
  nh.getParam("/trajectory_controller/I_zz", this->Izz);
  nh.getParam("/trajectory_controller/dist_cog_prop", this->L);

  this->g = 9.81;
}

void trajectory_controller::set_path(const nav_msgs::Path::ConstPtr& msg)
{
  current_path.header = msg->header;
  current_path.poses = msg->poses;
  current = current_path.poses[0];
  current_goal = current_path.poses[1];
}

void trajectory_controller::set_current_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
  current.position = msg->position;
  current.orientation = msg->orientation;
}

void trajectory_controller::set_current_goal(const geometry_msgs::Pose::ConstPtr& msg)
{
  current_goal.position = msg->position;
  current_goal.orientation = msg->orientation;
}

void trajectory_controller::set_current_rotations(const sensor_msgs::Imu::ConstPtr& msg)
{
  float x_imu = msg->angular_velocity.x;
  float y_imu = msg->angular_velocity.y;
  psi_dot = msg->angular_velocity.z;

  float root2 = sqrt(1/2);

  phi_dot = root2 * (x_imu + y_imu);
  theta_dot = root2 * (x_imu - y_imu);

void trajectory_controller::calc_controller_error()
{
  e_psi = K_D * psi_dot + K_P * psi + K_I * (psi - last_psi) * dt;
  e_phi = K_D * phi_dot + K_P * phi + K_I * (phi - last_phi) * dt;
  e_theta = K_D * theta_dot + K_P * theta + K_I * (theta - last_theta) * dt;
}

void transform_quaternion()
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
    ros::Subscriber path_sub = nh.subscribe("/phx/path", 1, &trajectory_controller::set_path, &controller);

    //ros::Subscriber init_sub = nh.subscribe("/phx/pose", 1, &trajectory_controller::set_current_pose, &controller);
    ros::Subscriber imu_pose = nh.subscribe("/phoenix/imu", 10, &trajectory_controller::set_current_rotations, &controller);
    //ros::Publisher MotorMsg = nh.advertise<>("/phoenix/cmd_motor", 10);

    ros::Rate loop_rate(50);


    while(ros::ok())
    {

        transform_quaternion();
        calc_controller_error();




        last_theta = theta;
        last_phi = phi;
        last_psi = psi;

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
