# pragma once

#include <systemc.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdbool.h>

// IDLE, RUNNING, STOP

/*A
 * PID controller for angular displacement of the robotic arm. 
 * Recieves as input the current displacement of the arm
 * red by a sensor, and outputs a singal to the motor that drives
 * the arm towards the desired target value.
 */
SC_MODULE(controller) {
	//Ports
	sc_in <bool> clk;
	sc_in <bool> reset;
	sc_out <sc_uint <4> > outLeds;

	//outputs
	sc_out <sc_int <16 > > out_ctl_motor_tau;

    //inputs
    sc_in <sc_int <16 > > in_snr_q;
    // sc_in <sc_int <16 > > in_q_target;

	//time
	float dt;				// Delta time [s]
	float time;				// Time [s]

	//constants
    float K_p;     			// Proportional gain [None]
    float K_i;				// Integral gain [None]
    float K_d;     			// Derivative gain [None]
	float threshold;

	//variables
    float q;		        // Angular displacement [rad]
	float q_target;			// Target angular displacement [rad]
	float q_error;			// Angular error signal [rad]
    float q_error_prev;  	// Previous error [rad]
    float q_error_deriv; 	// The rate of change of the error signal [rad]
    float q_error_integ; 	// Error signal integrated [rad]
	float ctl_motor_tau;	// Torque from the motor [N*m]
	sc_uint <4> led_counter;
    
	//states
	enum state {IDLE, RUNNING, STOP}; 
	state s;

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


		// INITIALIZING VALUES________________________________________
		//time
		dt = 0.0;
		time = 0.0;

		//constants
		K_p = 22.;
		K_i = 20.0;
		K_d = 7.;
		threshold = 0.01;

		//variables
		q = 0.0;
		q_target = 1.;
		q_error = 0.0;
		q_error_prev = 0.0;
		q_error_deriv = 0.0;
		q_error_integ = 0.0;
		ctl_motor_tau = 0.0;

		s = RUNNING;

		//Process Registration
		SC_CTHREAD(step, clk.pos());
		reset_signal_is(reset, true);
	}
};
