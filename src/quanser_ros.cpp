#include "quanser_ros.h"
#include <ros/ros.h>
//extern t_version version;
//extern t_card board;

t_card board;
t_version version;

t_int32 encoder_counts[NUM_ENCODER_CHANNELS];
t_double encoder_counts_per_sec[NUM_ENCODER_CHANNELS];

//ros float32 type
float encoder_rad[NUM_ENCODER_CHANNELS];
float encoder_rad_per_sec[NUM_ENCODER_CHANNELS];





void quanser_initialize(void)
{
    t_error  result;
    /*Encoder initial settings*/
    result = hil_set_encoder_quadrature_mode(board, encoder_channels,  NUM_ENCODER_CHANNELS, quadrature_mode);
    ROS_INFO("Result code %i: ", result);
    result = hil_set_encoder_filter_frequency(board, encoder_channels,  NUM_ENCODER_CHANNELS, encoder_filter_frequency);
    ROS_INFO("Result code %i: ", result);
}



t_error open_quanser(void)
{
    t_error result;
    result = hil_open(board_type, board_identifier, &board);
    return result;
}

/*Analog functionalities*/
void init_inputs(void)
{
    t_uint channel;
        // initialize voltages
        for (channel = 0; channel < NUM_ANALOG_OUTPUT_CHANNELS; channel++)
        {
            analog_initial_voltages[channel] = 5.0;
            analog_final_voltages[channel] = 0.0;
        }
            
}

t_error write_analog()
{
    t_error result;
    result = hil_write_analog(board, analog_output_channels, NUM_ANALOG_OUTPUT_CHANNELS, analog_initial_voltages);
    return result;
}
/*Encoder Functionalities*/

t_error read_encoder(void)
{
    ROS_INFO("Enter read_encoder");
    t_error result_pos, result_vel;
    //position
    result_pos = hil_read_encoder(board, encoder_channels, NUM_ENCODER_CHANNELS, &encoder_counts[0]);
    //velocity
    result_vel = hil_read_other(board,  encoder_vel_channels, NUM_ENCODER_CHANNELS, &encoder_counts_per_sec[0]);   
    ROS_INFO("Result code %i: ", result_pos);
    ROS_INFO("Result code %i: ", result_vel);

    if(result_pos >= 0 && result_vel >= 0)
    {
        //unit conversion from ticks to rad
        for(t_uint i = 0; i < NUM_ENCODER_CHANNELS; i++)
        {
            encoder_rad[i] = float(encoder_counts[i] * TICKS_TO_RAD);
            encoder_rad_per_sec[i] = float(encoder_counts_per_sec[i] * TICKS_TO_RAD); 
        }
        t_uint32 channel;
        for(channel = 0; channel < NUM_ENCODER_CHANNELS; channel++)
        {
            ROS_INFO("Encoder Position Reading %i: %5f \n" , encoder_channels[channel], encoder_rad[channel]); 
            ROS_INFO("Encoder Velocity Reading %i: %5f \n" , encoder_vel_channels[channel], encoder_rad_per_sec[channel]); 
        }
        return 0;
    }
    else
    {
        ROS_INFO("Error reading encoders");
    }

  return  -1;
}




void close_quanser(void)
{   
    t_error result;
    result = hil_write_analog(board, analog_output_channels, NUM_ANALOG_OUTPUT_CHANNELS, analog_final_voltages);
    hil_close(board);
}

void print_quanser_version(void)
{
    t_error result;
    version.size = sizeof(version);
    result = hil_get_version(&version);
    ROS_INFO("%i", result);
}
