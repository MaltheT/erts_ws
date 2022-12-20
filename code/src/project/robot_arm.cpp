#include "robot_arm.h"

void robot_arm::step()
{
	wait();
	while (1)
	{
		wait();
		//dt = current_t - last_t; for dynamic time calculation
		motor_tau = in_motor_tau.read();
		
		g_tau = m * g * r * cos(q);
		F_tau = F_c * sign(q_dot) + F_v * q_dot;
		q_ddot = (motor_tau + g_tau - F_tau)/interia;
		q_dot = q_dot + q_ddot * dt;
		q = q + q_dot * dt;

		out_q.write(q);
		// write q to file
		myfile << q << "\n";
	}
}



int robot_arm::sign(float x){
	if(x<0)
		return 0;
	else
		return 1;
}
