# pragma once

#include <systemc.h>
#include <math.h>
#include <iostream>
#include <fstream>
using namespace std;

/*
 * Robotic arm simulation. 
 * This module simulates the motion of of the robotic arm in 2D space.
 * The robotic arm recieves as input the torque generated by the motor.
 * The output of the module is the angular displacement of the arm, 
 * which is read by a sensor. 
 */
SC_MODULE(robot_arm) {
	//Ports
	sc_in <bool> clk;

	//time
	float dt;	 			// Delta time [s]
	float time; 			// Time [s]

	//constants
	float  g; 		// Gravity [m/s²]
	float  r; 		// Length of robot arm [m]
	float  F_c; 		// Coulomb friction factor [None]
	float  F_v; 	// Viscous friction factor [None]
	float  m; 		// Mass of end effector [kg]

	//variables
	float q_ddot;			// Angular acceleartion [rad/s²]
	float q_dot;			// Angular velocity [rad/s]
	float q;				// Angular displacement [rad]
	float g_tau;			// Torque due to gravity [N*m]
	float F_tau;			// Torque due to friction [N*m]
	float interia;	// Mass moment of inertia [kg*m²]
	float motor_tau; 		// Torque from the motor [N*m]

	//inputs
	sc_in<float> in_motor_tau;

	//outputs
	sc_out<float> out_q;

	//file
	ofstream myfile;

	void step();

	//Constructor
	SC_CTOR(robot_arm) {

		// Init time
		dt = 0.0;	 			// Delta time [s]
		time = 0.0; 			// Time [s]


		// Initializing constants
		g = -9.81; 		// Gravity [m/s²]
		r = 0.8; 		// Length of robot arm [m]
		F_c = 0.1; 		// Coulomb friction factor [None]
		F_v = 0.05; 	// Viscous friction factor [None]
		m = 1.2; 		// Mass of end effector [kg]


		// Initializing variables
		q_ddot = 0.0;
		q_dot = 0.0;
		q = 0.0;
		g_tau = 0.0;
		F_tau = 0.0;
		interia = m * r*r;
		motor_tau = 0.0;
		
		myfile.open ("output.txt");
		//Process Registration
		SC_CTHREAD(step, clk.pos());
	}

	~robot_arm(){
		myfile.close();
	}
};
