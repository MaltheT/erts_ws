#pragma once

#include <systemc.h>

SC_MODULE(controller_driver) {

	//Ports
	sc_in <bool> clk;
	sc_out <bool> reset;
	sc_out <float> out_q_target;

	int retval;

	//Process Declaration
	void test();

	//Constructor
	SC_CTOR(controller_driver) : retval(-1) {

		//Process Registration
		SC_CTHREAD(test, clk.pos());
		reset_signal_is(reset, true);
		//sensitive << clk.pos();
	}
};
