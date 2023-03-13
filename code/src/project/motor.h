#pragma once

#include <iostream>
#include <systemc.h>
#include <math.h>


/*
 * Motor actuator for the robotic arm joint.
 * Recieves as input a motor torque signal from the PID controller,
 * and actuates the robotic arm joint. 
 */
SC_MODULE(motor) {
	//Ports
	sc_in <bool> clk;
	sc_in<sc_int <16 > > in_ctl_motor_tau;
	sc_out<float> out_motor_tau;

	//variables
	float motor_tau;		// Torque from the motor [N*m]

	void step(){
		wait();
		while (1)
		{
			wait();
			motor_tau = ((float)in_ctl_motor_tau.read())/100000;
			//std::cout << "In value: " << ((float)in_ctl_motor_tau.read())/1000 << std::endl;
			//std::cout << motor_tau << std::endl;
			out_motor_tau.write(motor_tau);
		}
	}

	//Constructor
	SC_CTOR(motor){

		motor_tau = 0.0;
		//Process Registration
		SC_CTHREAD(step, clk.pos());
	}
};
