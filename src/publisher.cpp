#include "ros/ros.h"
#include "std_msgs/String.h"
#include "my_lib.h"
#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"
#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  t_error result;
  
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    
    result = open_quanser();
    
    init_inputs();

    if (result == 0)
    {
      ROS_INFO("Quanser board open!!!!!");
      
        result = write_analog();
        if (result >= 0)
        {
          ROS_INFO("Writing succesfull!!!!!");
        }

        read_encoder();
      
    }
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  close_quanser();

  return 0;
}