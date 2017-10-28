#include "ros/ros.h"
#include "phx_uart_msp_bridge/RemoteControl.h"
#include "math.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ssh_remote_control");

  ros::NodeHandle n;

  phx_uart_msp_bridge::RemoteControl rc;

  rc.pitch = 0; // integer!!
  rc.roll = 0; // 1 entspricht 0.25 grad
  rc.yaw = 0;
  rc.throttle = 0;
  rc.aux1 = 2; // 1 entspricht 10 cm
  rc.aux2 = 0;
  rc.aux3 = 0;
  rc.aux4 = 0;

  rc.header.stamp = ros::Time::now();

  ros::Rate loop_rate(10);
  ros::Publisher rc_pub = n.advertise<phx_uart_msp_bridge::RemoteControl>("/phx/ssh_rc", 1);

  char str[2] = {0};
  char* cmd = str;

  while (ros::ok()){
    cmd = fgets(cmd, 2, stdin);

    //angles
    if((strcmp(cmd, "i")) && rc.pitch > (-14.75/0.25))
    {
      rc.pitch -= 1;
    }
    if((strcmp(cmd, "k")) && rc.pitch < (14.75/0.25))
    {
      rc.pitch += 1;
    }
    if((strcmp(cmd, "j")) && rc.pitch > (-14.75/0.25))
    {
      rc.roll -= 1;
    }
    if((strcmp(cmd, "l")) && rc.pitch < (14.75/0.25))
    {
      rc.roll += 1;
    }

    //altitude / thottle
    if((strcmp(cmd, "a")) && rc.aux1 > 0)
    {
      rc.aux1 -= 1;
    }
    if((strcmp(cmd, "q")) && rc.aux1 < 3)
    {
      rc.aux1 += 1;
    }

    // altitude cmd
    ROS_DEBUG("roll_cmd %lf [°]\t pitch_cmd %lf [°]\t altitude_cmd %lf [m]\n", rc.roll*1.0/4, rc.pitch*1.0/4, rc.aux1*1.0/10);

    // thottle cmd


    rc_pub.publish(rc);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;

}

