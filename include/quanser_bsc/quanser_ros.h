#ifndef QUANSER_ROS_H
#define QUANSER_ROS_H
//#define _USE_MATH_DEFINES
#include <cmath>
#include <ros/ros.h>

#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"

/*Board configuration*/
static const char board_type[]       = "qpid_e";
static const char board_identifier[] = "0";
static char       message[512];

/*Analog configuration*/
//Analog channels
const t_uint32 analog_input_channels[]    = { 0, 1, 2, 3 };
const t_uint32 analog_output_channels[]   = { 0, 1, 2, 3 };
//Encoder channels
const t_uint32 encoder_channels[] = {1, 2};
const t_uint32 encoder_vel_channels[] = {14001, 14002};
const t_encoder_quadrature_mode quadrature_mode[] = {ENCODER_QUADRATURE_4X, ENCODER_QUADRATURE_4X};
const t_double encoder_filter_frequency[] = {1/(40e-6), 1/(40e-6)};
/*Static*/
//Analog
#define NUM_ANALOG_INPUT_CHANNELS      ARRAY_LENGTH(analog_input_channels)
#define NUM_ANALOG_OUTPUT_CHANNELS     ARRAY_LENGTH(analog_output_channels)
//Encoders
#define NUM_ENCODER_CHANNELS           ARRAY_LENGTH(encoder_channels)
#define TICKS_TO_RAD                 2*M_PI/20000 
#define SAMPLES                 500                             /* 0.5 seconds worth of SAMPLES */

//analog vectors
static t_double analog_initial_voltages[NUM_ANALOG_OUTPUT_CHANNELS];
static t_double analog_input_voltages[SAMPLES][NUM_ANALOG_INPUT_CHANNELS];
static t_double analog_output_voltages[SAMPLES][NUM_ANALOG_OUTPUT_CHANNELS];
static t_double analog_final_voltages[NUM_ANALOG_OUTPUT_CHANNELS];


const t_double frequency           = 1000;
const t_double sine_frequency      = 10; /* frequency of output sinewaves */
const t_double period              = 1.0 / frequency;

void quanser_initialize(void);

t_error open_quanser(void);

t_error write_analog();

void close_quanser(void);

void init_inputs(void);

void print_quanser_version(void);

t_error read_encoder(void);



#endif