#pragma once

#define SC_INCLUDE_FX 
#include <systemc.h>
#include <math.h>


/*
 * Motor actuator for the robotic arm joint.
 * Recieves as input a motor torque signal from the PID controller,
 * and actuates robotic arm. 
 */
SC_MODULE(motor) {
	//Ports
	sc_in <bool> clk;
	sc_in<float> in_ctl_motor_tau;
	sc_out<float> out_motor_tau;

	//variables
	float motor_tau = 0.0;		// Torque from the motor [N*m]

	void step(){
		wait();
		while (1)
		{
			wait();
			motor_tau = in_ctl_motor_tau.read();
			out_motor_tau.write(motor_tau);
		}
	}

	//Constructor
	SC_CTOR(motor){

		//Process Registration
		SC_CTHREAD(step, clk.pos());
	}
};
