#ifndef QUANSER_BOARDS_H
#define QUANSER_BOARDS_H
#include <cmath>
#include <ros/ros.h>
#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"

#define TICKS_TO_RAD                 2*M_PI/20000 

namespace quanser
{
    namespace qpid_e{
        static const char board_type[]= "qpid_e";
        static const char board_identifier[]= "0";
        const t_uint32 encoder_channels[] = {1, 2};
        const t_uint32 encoder_vel_channels[] = {14001, 14002};
        const t_encoder_quadrature_mode quadrature_mode[] = {ENCODER_QUADRATURE_4X, ENCODER_QUADRATURE_4X};
        const t_double encoder_filter_frequency[] = {1 /(120e-9 * 1), 1 /(120e-9 * 1)};
        #define QPIDE_NUM_ENCODER_CHANNELS      ARRAY_LENGTH(encoder_channels) 

    };

    class QuanserBoard
    {   
        private:
            
            t_version version;
            t_int32 *encoder_counts;
            t_double *encoder_counts_per_sec;
            t_uint32 num_encoder_channels;
            t_card board;

        public:
            float *encoder_rad;
            float *encoder_rad_per_sec;
            

            QuanserBoard();

            t_error open_quanser(void);

            void close_quanser(void);

            void quanser_initialize(void);

            t_error read_encoder(void);
            
    };



};

#endif