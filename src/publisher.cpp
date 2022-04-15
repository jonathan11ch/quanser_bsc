#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
//#include "quanser_ros.h"
#include "quanser_boards.h"
#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"
#include <sstream>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("angles", 1000);
  ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("vel", 1000);

  ros::Rate loop_rate(1);

  t_error result;
  int count = 0;
  quanser::QuanserBoard quanser_board;
    

  

  
  ROS_INFO("Quanser opening...");
  result = quanser_board.open_quanser();
  if (result < 0){
    ROS_INFO("Quanser cannot open");
    return -1;
  }
  ROS_INFO("Quanser board open!!!!!");
    ROS_INFO("Quanser initialize...");
  quanser_board.quanser_initialize();
  //quanser_initialize();
  while (ros::ok())
  {
    std_msgs::String msg;
    
    //init_inputs(); 
    //result = write_analog();
    if (result >= 0)
    {
      ROS_INFO("Writing succesfull!!!!!");
    }
    ROS_INFO("reading encoders...");
    result = quanser_board.read_encoder();

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());
    //extern float encoder_rad[NUM_ENCODER_CHANNELS];
    //extern float encoder_rad_per_sec[NUM_ENCODER_CHANNELS];
    float angles, vel;
    angles = quanser_board.encoder_rad[0];
    vel = quanser_board.encoder_rad_per_sec[0];

    angle_pub.publish(angles);
    vel_pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  quanser_board.close_quanser();

  return 0;
}