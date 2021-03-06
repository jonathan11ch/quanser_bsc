#include "quanser_boards.h"
#include <ros/ros.h>



namespace quanser
{
    /*constructor*/
    QuanserBoard::QuanserBoard()
    {
        ROS_INFO("Quanser board object created");
        /*Buffers*/
        num_encoder_channels  = 2;
        ROS_INFO("Number of encoder channels %i: ", num_encoder_channels);
        encoder_rad = new float[num_encoder_channels];
        encoder_rad_per_sec = new float[num_encoder_channels];
        encoder_counts = new t_int32[num_encoder_channels];
        encoder_counts_per_sec = new t_double[num_encoder_channels];

    }

    /*Open board method*/
    t_error QuanserBoard::open_quanser(void)
    {
        ROS_INFO("Board type %s:" , qpid_e::board_type);
        ROS_INFO("Board identifier %s: ", qpid_e::board_identifier);
        t_error result;
        result = hil_open(qpid_e::board_type, qpid_e::board_identifier, &(this->board));
        return result;
    }

    /*Close board method*/
    void QuanserBoard::close_quanser(void)
    {   
        hil_close(this->board);
    }

    void QuanserBoard::quanser_initialize(void)
    {
        t_error result;
        /*Encoder initial settings*/
          
        ROS_INFO("enter initialize...");
        result = hil_set_encoder_quadrature_mode(this->board, qpid_e::encoder_channels,  this->num_encoder_channels, qpid_e::quadrature_mode);
        ROS_INFO("Result code %i: ", result);
        result = hil_set_encoder_filter_frequency(this->board, qpid_e::encoder_channels,  this->num_encoder_channels, qpid_e::encoder_filter_frequency);
        ROS_INFO("Result code %i: ", result);
    }
    /**/
    t_error QuanserBoard::read_encoder(void)
    {
        ROS_INFO("Enter read_encoder");
        t_error result_pos, result_vel;
        //position
        result_pos = hil_read_encoder(this->board, qpid_e::encoder_channels, this->num_encoder_channels, &(this->encoder_counts[0]));
        //velocity
        result_vel = hil_read_other(this->board,  qpid_e::encoder_vel_channels, this->num_encoder_channels, &(this->encoder_counts_per_sec[0]));   
        //unit conversion from ticks to rad
        for(t_uint i = 0; i < num_encoder_channels; i++){
            encoder_rad[i] = float(encoder_counts[i] * TICKS_TO_RAD);
            encoder_rad_per_sec[i] = float(encoder_counts_per_sec[i] * TICKS_TO_RAD); 
        }

        if(result_pos >= 0 && result_vel >= 0)
        {
            t_uint32 channel;
            for(channel = 0; channel < num_encoder_channels; channel++)
            {
                ROS_INFO("Encoder Position Reading %i: %5f \n" , qpid_e::encoder_channels[channel], encoder_rad[channel]); 
                ROS_INFO("Encoder Velocity Reading %i: %5f \n" , qpid_e::encoder_vel_channels[channel], encoder_rad_per_sec[channel]); 
            }
            return 0;
        }
        else
        {
            ROS_INFO("Error reading encoders");
        }

    return  -1;
    }        
}
