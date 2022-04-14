#include "quanser_ros.h"

//extern t_version version;
//extern t_card board;

t_card board;
t_version version;

t_int32 encoder_counts[NUM_ENCODER_CHANNELS];

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
    t_error result;
    result = hil_read_encoder(board, encoder_channels, NUM_ENCODER_CHANNELS, &encoder_counts[0]);
    if(result >= 0)
    {
        t_uint32 channel;
        for(channel = 0; channel < NUM_ENCODER_CHANNELS; channel++)
        {
            ROS_INFO("Encoder Reading %i: %5i \n" , encoder_channels[channel], encoder_counts[channel]); 
        }
    }
    else
    {
        ROS_INFO("Error reading encoders");
    }
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