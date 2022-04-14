#ifndef MY_LIB_H
#define MY_LIB_H
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
const t_uint32 encoder_channels[] = {0, 1};

/*Static*/
//Analog
#define NUM_ANALOG_INPUT_CHANNELS      ARRAY_LENGTH(analog_input_channels)
#define NUM_ANALOG_OUTPUT_CHANNELS     ARRAY_LENGTH(analog_output_channels)
//Encoders
#define NUM_ENCODER_CHANNELS           ARRAY_LENGTH(encoder_channels)

#define SAMPLES                 500                             /* 0.5 seconds worth of SAMPLES */

//analog vectors
static t_double analog_initial_voltages[NUM_ANALOG_OUTPUT_CHANNELS];
static t_double analog_input_voltages[SAMPLES][NUM_ANALOG_INPUT_CHANNELS];
static t_double analog_output_voltages[SAMPLES][NUM_ANALOG_OUTPUT_CHANNELS];
static t_double analog_final_voltages[NUM_ANALOG_OUTPUT_CHANNELS];


const t_double frequency           = 1000;
const t_double sine_frequency      = 10; /* frequency of output sinewaves */
const t_double period              = 1.0 / frequency;


t_error open_quanser(void);

t_error write_analog();

void close_quanser(void);

void init_inputs(void);

void print_quanser_version(void);

t_error read_encoder(void);



#endif