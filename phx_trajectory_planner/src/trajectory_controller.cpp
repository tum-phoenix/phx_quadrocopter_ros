// implements a pid_controller as modeled in the paper "Quadrocopter Dynamics, Simulation and Control"

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

geometry_msgs::Pose trajectory_controller::find_nearest_pose(const geometry_msgs::Pose::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle nh;

    trajectory_controller controller(nh); // init class

    // subscribe to class trajectory_controller controller's method set_path
    ros::Subscriber path_sub = nh.subscribe("/phx/path", 1, &trajectory_controller::set_path, &controller);

    ros::Subscriber init_sub = nh.subscribe("/phx/pose", 1, &trajectory_controller::set_current_pose, &controller);

    ros::Rate loop_rate(50);

    while(ros::ok())
    {


        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
