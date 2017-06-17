// implements a pid_controller as modeled in the paper "Quadrocopter Dynamics, Simulation and Control"

#include "phx_trajectory_planner/trajectory_controller.h"

trajectory_controller::trajectory_controller(ros::NodeHandle nh)
{
  // get Parameters specified in launchfile
  // all in SI units
  m = 0;
  k = 0;
  b = 0;
  Ixx = 0;
  Iyy = 0;
  Izz = 0;
  L = 0; // distance from cog to any of the propellers

  nh.getParam("/pid_controller/mass", m);
  nh.getParam("/pid_controller/thrust_rpm_const_k", k);
  nh.getParam("/pid_controller/torque_drag_const_b", b);
  nh.getParam("/pid_controller/I_xx", Ixx);
  nh.getParam("/pid_controller/I_yy", Iyy);
  nh.getParam("/pid_controller/I_zz", Izz);
  nh.getParam("/pid_controller/dist_cog_prop", L);

  g = 9.81;
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_controller");
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
