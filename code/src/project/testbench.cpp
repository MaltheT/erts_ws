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
	

using namespace sc_core;


int sc_main (int argc , char *argv[])
{
	// sc_report_handler::set_actions("/IEEE_Std_1666/deprecated", SC_DO_NOTHING);
	// sc_report_handler::set_actions( SC_ID_LOGIC_X_TO_BOOL_, SC_LOG);
	// sc_report_handler::set_actions( SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, SC_LOG);
	// sc_report_handler::set_actions( SC_ID_OBJECT_EXISTS_, SC_LOG);

	// sc_set_time_resolution(1, SC_FS);
	// sc_set_default_time_unit(1, SC_SEC);
	
	sc_trace_file *tracefile;

	sc_signal<bool> s_reset;
	sc_signal<float> s_q;
	sc_signal<float> s_motor_tau;
	sc_signal<sc_int <16 > > s_snr_q;
	sc_signal<sc_int <16 > > s_ctl_motor_tau;
	sc_signal<sc_uint<4> > s_leds;

	//Creating clocks for all components of the system
	sc_clock s_robot_arm_clk("s_robot_arm_clk", 	0.2, SC_MS);
	sc_clock s_motor_clk("s_motor_clk", 			0.2, SC_MS);
	sc_clock s_sensor_clk("s_sensor_clk", 			0.2, SC_MS);
	sc_clock s_controller_clk("s_controller_clk", 	0.2, SC_MS);

	robot_arm RobotArm("robot_arm");
	motor Motor("motor");
	sensor Sensor("sensor");
	controller Controller("controller");
	controller_driver ControllerDriver("controller_driver");

	// Create tacefile
	tracefile = sc_create_vcd_trace_file("IOSC_Wave");
	if (!tracefile) cout << "Could not create trace file." << endl;

	// Set resolution of trace file to be in 10 US
	tracefile->set_time_unit(100, SC_NS);

	sc_trace(tracefile, s_robot_arm_clk,	"s_robot_arm_clk");
	sc_trace(tracefile, s_motor_clk,    	"s_motor_clk");
	sc_trace(tracefile, s_sensor_clk,    	"s_sensor_clk");
	sc_trace(tracefile, s_controller_clk,   "s_controller_clk");
	sc_trace(tracefile, s_leds,   			"leds");

	sc_trace(tracefile, s_reset,  			"reset");
	sc_trace(tracefile, s_q,   				"q");
	sc_trace(tracefile, s_snr_q,   			"snr_q");
	sc_trace(tracefile, s_motor_tau,		"motor_tau");
	sc_trace(tracefile, s_ctl_motor_tau,	"ctl_motor_tau");


	// Connect the signals to ports
	RobotArm.clk(s_robot_arm_clk);
	RobotArm.out_q(s_q);
	RobotArm.in_motor_tau(s_motor_tau);

	Motor.clk(s_motor_clk);
	Motor.in_ctl_motor_tau(s_ctl_motor_tau);
	Motor.out_motor_tau(s_motor_tau);

	Sensor.clk(s_sensor_clk);
	Sensor.in_q(s_q);
	Sensor.out_snr_q(s_snr_q);

	Controller.clk(s_controller_clk);
	Controller.reset(s_reset);
	Controller.in_snr_q(s_snr_q);
	Controller.out_ctl_motor_tau(s_ctl_motor_tau);
	Controller.outLeds(s_leds);

	ControllerDriver.clk(s_controller_clk);
	ControllerDriver.reset(s_reset);

	// Sim for 200
	int end_time = 6000;
	std::cout << "INFO: Simulating" << std::endl;
	// start simulation
	sc_start(end_time, SC_MS);
	sc_close_vcd_trace_file(tracefile);
	std::cout << "Created IOSC_Wave.vcd" << std::endl;

	return 0;
};
