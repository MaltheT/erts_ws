# pragma once

#define SC_INCLUDE_FX 
#include <systemc.h>
#define _USE_MATH_DEFINES
#include <math.h>


/*
 * PID controller for angular displacement of the robotic arm. 
 * Recieves as input the current displacement of the arm
 * red by a sensor, and outputs a singal to the motor that drives
 * the arm towards the desired target value.
 */
SC_MODULE(controller) {
	//Ports
	sc_in <bool> clk;
	sc_in <bool> reset;

	//constants
	float const dt = 0.1; 		// Times step [s]
    float const K_p = 2.0;     	// Proportional gain [None]
    float const K_i = 0.5;     	// Integral gain [None]
    float const K_d = 1.0;     	// Derivative gain [None]

	//variables
    float q = 0.0;		        // Angular displacement [rad]
	float q_target = 1.;		// Target angular displacement [rad]
	float q_error = 0.0;		// Angular error signal [rad]
    float q_error_prev = 0.0;  	// Previous error [rad]
    float q_error_deriv = 0.0; 	// The rate of change of the error signal [rad]
    float q_error_integ = 0.0; 	// Error signal integrated [rad]
	float ctl_motor_tau = 0.0;	// Torque from the motor [N*m]
    
	//outputs
	sc_out<sc_fixed<4,4>> out_ctl_motor_tau;

    //inputs
    sc_in<sc_fixed<4,4>> in_snr_q;

	void step();

	//Constructor
	SC_CTOR(controller){

		//Process Registration
		SC_CTHREAD(step, clk.pos());
		reset_signal_is(reset, true);
	}
};
