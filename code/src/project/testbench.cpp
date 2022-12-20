#define SC_INCLUDE_FX 
#include <systemc.h>
#include <stdio.h>
#include "robot_arm.h"
#include "sensor.h"
#include "motor.h"
#include "controller_driver.h"

#ifdef __RTL_SIMULATION__
	#include "controller_rtl_wrapper.h"
	#define controller controller_rtl_wrapper
#else
	#include "controller.h"
#endif


int sc_main (int argc , char *argv[])
{
	// sc_report_handler::set_actions("/IEEE_Std_1666/deprecated", SC_DO_NOTHING);
	// sc_report_handler::set_actions( SC_ID_LOGIC_X_TO_BOOL_, SC_LOG);
	// sc_report_handler::set_actions( SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, SC_LOG);
	// sc_report_handler::set_actions( SC_ID_OBJECT_EXISTS_, SC_LOG);

	sc_trace_file *tracefile;

	sc_signal<bool> s_reset;
	sc_signal<float> s_q;
	sc_signal<float> s_motor_tau;
	sc_signal<float> s_snr_q;
	sc_signal<float> s_ctl_motor_tau;
	sc_signal<float> s_q_target;

	// Create a 10ns period clock signal
	sc_clock s_clk("s_clk", 10, SC_NS);

	robot_arm RobotArm("robot_arm");
	motor Motor("motor");
	sensor Sensor("sensor");
	controller Controller("controller");
	controller_driver ControllerDriver("controller_driver");

	// Create tacefile
	tracefile = sc_create_vcd_trace_file("IOSC_Wave");
	if (!tracefile) cout << "Could not create trace file." << endl;

	// Set resolution of trace file to be in 10 US
	tracefile->set_time_unit(1, SC_NS);

	sc_trace(tracefile, s_clk,    		"clock");
	sc_trace(tracefile, s_reset,  		"reset");
	sc_trace(tracefile, s_q,   			"q");
	sc_trace(tracefile, s_snr_q,   		"snr_q");
	sc_trace(tracefile, s_motor_tau,	"motor_tau");
	sc_trace(tracefile, s_ctl_motor_tau,"ctl_motor_tau");
	sc_trace(tracefile, s_q_target, 	"q_target");


	// Connect the signals to ports
	RobotArm.clk(s_clk);
	RobotArm.out_q(s_q);
	RobotArm.in_motor_tau(s_motor_tau);

	Motor.clk(s_clk);
	Motor.in_ctl_motor_tau(s_ctl_motor_tau);
	Motor.out_motor_tau(s_motor_tau);

	Sensor.clk(s_clk);
	Sensor.in_q(s_q);
	Sensor.out_snr_q(s_snr_q);

	Controller.clk(s_clk);
	Controller.reset(s_reset);
	Controller.in_snr_q(s_snr_q);
	Controller.out_ctl_motor_tau(s_ctl_motor_tau);
	Controller.in_q_target(s_q_target);

	ControllerDriver.clk(s_clk);
	ControllerDriver.out_q_target(s_q_target);
	ControllerDriver.reset(s_reset);

	// Sim for 200
	int end_time = 6000*10;
	std::cout << "INFO: Simulating" << std::endl;
	// start simulation
	sc_start(end_time, SC_NS);
	sc_close_vcd_trace_file(tracefile);
	std::cout << "Created IOSC_Wave.vcd" << std::endl;

	return 0;
};
