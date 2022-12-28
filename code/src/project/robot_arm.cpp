#include "robot_arm.h"

void robot_arm::step()
{
	wait();
	while (1)
	{
		wait();
		motor_tau = in_motor_tau.read();

		dt = sc_time_stamp().to_seconds() - time;		// Dynamic delta time calculation
		time = sc_time_stamp().to_seconds();
		
		// Advance states:
		g_tau = m * g * r * cos(q);
		F_tau = F_c * tanh(q_dot) + F_v * q_dot;
		q_ddot = (motor_tau + g_tau - F_tau)/interia;
		q_dot = q_dot + q_ddot * dt;
		q = q + q_dot * dt;

		out_q.write(q);
		// write q to file
		myfile << q << "\n";
	}
}

