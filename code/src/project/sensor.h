#pragma once

#define SC_INCLUDE_FX 
#include <systemc.h>
#include <math.h>

SC_MODULE(sensor) {

	//Ports
	sc_in <bool> clk;

	//input
	sc_in<sc_fixed<4,4>> in_q;

	//output
	sc_out<sc_fixed<4,4>> out_snr_q;

	//variables
	sc_fixed<4,4> snr_q = 0.0;		// Angular displacement [rad]


	void step(){
		wait();
		while (1)
		{
			wait();
			snr_q = in_q.read(); 
			out_snr_q.write(snr_q);
		}
	}

	//Constructor
	SC_CTOR(sensor){

		//Process Registration
		SC_CTHREAD(step, clk.pos());
	}
};
