#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "cmath"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//For testing
#include "cstdlib"

geometry_msgs::PoseStamped initial;
geometry_msgs::PoseStamped goal;

void initialCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  initial.header = msg->header;
  initial.pose = msg->pose;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goal.header = msg->header;
  goal.pose = msg->pose;
}

// returns array of float64s of difference vector between Poses initial and goal: [x,y,z] (not Quaternions)
// initial and goal are poses
// dVector must be an array of doubles with 3 elements
// divides dVector's length into n equal pieces
void linear_pose_diff(geometry_msgs::Pose initial, geometry_msgs::Pose goal, double* dVector, int n)
{
    dVector[0] = (goal.position.x - initial.position.x)/n;
    dVector[1] = (goal.position.y - initial.position.y)/n;
    dVector[2] = (goal.position.z - initial.position.z)/n;
}

// adds x,y,z components of double[x,y,z] step_vector array to intermediate
void add_step(geometry_msgs::Pose* intermediate, double* step_vector)
{
    intermediate->position.x += step_vector[0];
    intermediate->position.y += step_vector[1];
    intermediate->position.z += step_vector[2];
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_planner");

  ros::NodeHandle nh;

  ros::Subscriber init_sub = nh.subscribe("/phx/pose", 1000, initialCallback);
  ros::Subscriber goal_sub = nh.subscribe("/phx/current_goal", 1000, goalCallback); //Alternative: "/move_base_simple/goal"

  ros::Publisher pathpub = nh.advertise<nav_msgs::Path>("/phx/path", 1000);

  // number of points to interpolate between goal and initial
  int num_pts = 0;
  nh.getParam("/path_planner/num_points", num_pts);

  // CHECKEN dass parameter ankommt
  //std::cout << "num_points: " << num_pts << std::endl;

  ros::Rate loop_rate(1);

  while (ros::ok()){
    nav_msgs::Path path;
    std::vector<geometry_msgs::PoseStamped> posestoadd;

    posestoadd.push_back(initial);

    if(num_pts > 0)
    {
        // Richtungsvektor ausrechnen
        double dVector[3] = {0};
        linear_pose_diff(initial.pose, goal.pose, dVector, num_pts);

        geometry_msgs::PoseStamped intermediate = initial;

        for(int i = 0; i < num_pts-1; i++)
        {
            // Von initial zu goal gradient addieren
            add_step(&(intermediate.pose), dVector);

            // und Rotationen mit SLERP interpolieren
            tf2::Quaternion initial_tf, goal_tf, temp;
            double step = (i+1)*1.000/num_pts;

            tf2::fromMsg(initial.pose.orientation, initial_tf);
            tf2::fromMsg(goal.pose.orientation, goal_tf);
            temp = initial_tf;
            temp = tf2::slerp(initial_tf, goal_tf, step);

            intermediate.pose.orientation = tf2::toMsg(temp);
            posestoadd.push_back(intermediate);

            //Debug
            //std::cout << "Step: " << step << ";temp.w= " << temp.getW() << ";goal_tf.w= " << goal_tf.getW() << std::endl;
        }
    }

    posestoadd.push_back(goal);

    path.poses = posestoadd;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    pathpub.publish(path);
    ros::spinOnce();


    loop_rate.sleep();
  }

  return 0;
}
