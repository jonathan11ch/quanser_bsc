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
  static const char board_type[]       = "qpid_e";
  static const char board_identifier[] = "0";
  static char       message[512];


  const t_uint32 input_channels[]    = { 0, 1, 2, 3 };
  const t_uint32 output_channels[]   = { 0, 1, 2, 3 };
  const t_double frequency           = 1000;
  const t_double sine_frequency      = 10; /* frequency of output sinewaves */
  const t_double period              = 1.0 / frequency;

  #define NUM_INPUT_CHANNELS      ARRAY_LENGTH(input_channels)
  #define NUM_OUTPUT_CHANNELS     ARRAY_LENGTH(output_channels)
  #define SAMPLES                 500                             /* 0.5 seconds worth of SAMPLES */
  static t_double initial_voltages[NUM_OUTPUT_CHANNELS];
  static t_double input_voltages[SAMPLES][NUM_INPUT_CHANNELS];
  static t_double output_voltages[SAMPLES][NUM_OUTPUT_CHANNELS];
  static t_double final_voltages[NUM_OUTPUT_CHANNELS];

  qsigaction_t action;
  t_card board;

  action.sa_handler = SIG_IGN;
  action.sa_flags   = 0;

  t_version version;
  t_error result;
  
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    //t_int  result;
    
    version.size = sizeof(version);
    result = hil_get_version(&version);
    ROS_INFO("%i", result);
    //qsigemptyset(&action.sa_mask);
    //qsigaction(SIGINT, &action, NULL);

    result = hil_open(board_type, board_identifier, &board);
    if (result == 0)
    {
      ROS_INFO("Quanser board open!!!!!");
        
        t_uint channel, index;
        // initialize voltages
        for (channel = 0; channel < NUM_OUTPUT_CHANNELS; channel++)
            initial_voltages[channel] = 5.0;
            final_voltages[channel] = 0.0;
        // 
        for (index = 0; index < SAMPLES; index++)
        {
            t_double time = index * period;
            for (channel = 0; channel < NUM_OUTPUT_CHANNELS; channel++)
                output_voltages[index][channel] = channel + 4;
        }
        ROS_INFO("Before writing!!!!!");
        result = hil_write_analog(board, output_channels, NUM_OUTPUT_CHANNELS, initial_voltages);
        if (result >= 0)
        {
          ROS_INFO("Writing succesfull!!!!!");
        }
      
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
  ROS_INFO("Shutting down");
  result = hil_write_analog(board, output_channels, NUM_OUTPUT_CHANNELS, final_voltages);
  hil_close(board);

  return 0;
}