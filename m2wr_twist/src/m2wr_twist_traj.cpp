#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "m2wr_twist_traj");
  ros::NodeHandle nh;
  ros::Rate loop_rate(5);

  ros::Publisher traj_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  geometry_msgs::Twist vel;
  vel.linear.x = 1.0;
  vel.angular.z = 1.0;

  while (ros::ok())
  {
    traj_pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
