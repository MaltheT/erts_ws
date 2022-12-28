# pragma once

#include <systemc.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdbool.h>

// IDLE, RUNNING, STOP

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
	sc_out<sc_uint<4> > out_leds;

	//outputs
	sc_out<float> out_ctl_motor_tau;

    //inputs
    sc_in<float> in_snr_q;

	//time
	float dt = 0.0;			 	// Delta time [s]
	float time = 0.0;			// Time [s]

	//constants
    float const K_p = 22.;     	// Proportional gain [None]
    float const K_i = 20.0;     // Integral gain [None]
    float const K_d = 7.;     	// Derivative gain [None]
	float const threshold = 0.01; 

	//variables
    float q = 0.0;		        // Angular displacement [rad]
	float q_target = 1.;		// Target angular displacement [rad]
	float q_error = 0.0;		// Angular error signal [rad]
    float q_error_prev = 0.0;  	// Previous error [rad]
    float q_error_deriv = 0.0; 	// The rate of change of the error signal [rad]
    float q_error_integ = 0.0; 	// Error signal integrated [rad]
	float ctl_motor_tau = 0.0;	// Torque from the motor [N*m]
	sc_uint<4> led_counter;
    
	//states
	enum state {IDLE, RUNNING, STOP}; 
	state s = RUNNING;

	void step();
	void regulate();

	void change_angle(float new_q_target){
		if(s == IDLE){
			q_target = new_q_target;
			s = RUNNING;
		}
	}

	bool angle_reached(){
		if(abs(q - q_target) < threshold){
			return true;
		} 
		else {
			return false;
		}
	}

	void emergency_stop(){
		s = STOP;
	}

	void start(){
		if(s == STOP){
		s = RUNNING;
		}
	}

	//Constructor
	SC_CTOR(controller){

		// sc_set_default_time_unit(1, SC_SEC);
		//Process Registration
		SC_CTHREAD(step, clk.pos());
		reset_signal_is(reset, true);
	}
};
