#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->header.frame_id);
  std::cout << msg->ranges[0] << std::endl; 
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "mover_scans");
  ros::NodeHandle n;
  ros::Subscriber mover_sub = n.subscribe("/tb3_0/scan", 100, scanCallback);
  ros::spinOnce();
  
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::init(argc, argv, "mover_vels");
  ros::Publisher mover_pub = n.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 100);
  ros::spinOnce();
  
  int count = 0;
  while(ros::ok()){
    geometry_msgs::Twist send;
    send.linear.x = 0.2;
    send.angular.z = -.1;
    mover_pub.publish(send);
    ros::spinOnce();
    count++;
  }

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  return 0;
}
