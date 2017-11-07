#include "ros/ros.h"
#include "phx_uart_msp_bridge/RemoteControl.h"
#include "math.h"


#include <unistd.h>
#include <termios.h>

// see https://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ssh_remote_control");

  ros::NodeHandle n;

  phx_uart_msp_bridge::RemoteControl rc;

  rc.pitch = 0; // integer!!
  rc.roll = 0; // 1 entspricht 0.5 grad
  rc.yaw = 0;
  rc.throttle = 0;
  rc.aux1 = 0; // 1 entspricht 10 cm -- fuer Throttle Delta Mode auf 0 setzen!!
  rc.aux2 = 0;
  rc.aux3 = 0;
  rc.aux4 = 0;

  rc.header.stamp = ros::Time::now();

  ros::Rate loop_rate(10);
  ros::Publisher rc_pub = n.advertise<phx_uart_msp_bridge::RemoteControl>("/phx/ssh_rc", 1);

  char cmd[2] = {0};
  //char* cmd = str;

  //char cmd = 0;

  while (ros::ok()){
    //cmd = fgets(cmd, 1, stdin);

    cmd[0] = getch();

    //angles
    if((strcmp(cmd, "i") == 0) && (rc.pitch > (-10/0.5))) // strcmp returns 0 if strings are equal ... stupid C
    {
      rc.pitch--;
    }
    if((strcmp(cmd, "k") == 0) && (rc.pitch < (10/0.5)))
    {
      rc.pitch++;
    }
    if((strcmp(cmd, "j") == 0) && (rc.roll > (-10/0.5)))
    {
      rc.roll--;
    }
    if((strcmp(cmd, "l") == 0) && (rc.roll < (10/0.5)))
    {
      rc.roll++;
    }

    //altitude cmd mode --> aux1 auf 2 setzen! (0.2 m)
/*
    if((strcmp(cmd, "a") == 0) && rc.aux1 > 0)
    {
      rc.aux1--;
    }
    if((strcmp(cmd, "q") == 0) && rc.aux1 < 3*10)
    {
      rc.aux1++;
    }
*/
    
    // not aus
    if(strcmp(cmd, "x") == 0)
    {
      rc.aux2 = 1;
    }
    if(strcmp(cmd, "y") == 0)
    {
      rc.aux2 = 0;
    }

    //throttle cmd mode --> aux1 auf 0 setzen!
    if(strcmp(cmd, "a") == 0)
    {
      rc.aux1--;
    }
    if(strcmp(cmd, "q") == 0)
    {
      rc.aux1++;
    }

    //ROS_DEBUG("roll_cmd %lf [°]\t pitch_cmd %lf [°]\t altitude_cmd %lf [m]\n", rc.roll*1.0/4, rc.pitch*1.0/4, rc.aux1*1.0/10);

    // Altitude Comamnd Version
    //std::cout << "roll_cmd: " << rc.roll*1.0/2 << " [°] " << "pitch_cmd: " << rc.pitch*1.0/2 << " [°] " << "altitude_cmd: " << rc.aux1*1.0/10 << " [m]" << std::endl;

    // Thrust Delta Command Version
    std::cout << "roll_cmd: " << rc.roll*1.0/2 << " [°] " << "pitch_cmd: " << rc.pitch*1.0/2 << " [°] " << "Throttle_delta: " << rc.aux1 << " [%]" << std::endl;

    rc_pub.publish(rc);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;

}

