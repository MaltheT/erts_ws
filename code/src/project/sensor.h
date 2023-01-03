#pragma once

#include <systemc.h>
#include <math.h>

SC_MODULE(sensor) {

	//Ports
	sc_in <bool> clk;

	//input
	sc_in<float> in_q;

	//output
	sc_out<sc_int <16 > > out_snr_q;

	//variables
	float snr_q;		// Angular displacement [rad]


	void step(){
		wait();
		while (1)
		{
			wait();
			snr_q = in_q.read(); 
			out_snr_q.write(int(snr_q*1000));
		}
	}

	//Constructor
	SC_CTOR(sensor){

		snr_q = 0.0;
		//Process Registration
		SC_CTHREAD(step, clk.pos());
	}
};
