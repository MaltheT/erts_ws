#include "controller.h"

void controller::step()
{
	#pragma HLS resource core = AXI4LiteS metadata = "-bus_bundle slv0" variable = in_snr_q
	//
	#pragma HLS resource core = AXI4LiteS metadata = "-bus_bundle slv0" variable = out_ctl_motor_tau

	wait();

    step_label0:while(1)
    {
        wait();
        q = ((float)in_snr_q.read())/1000;
        //q = q / 1000;


        switch (s)
        {
        case RUNNING:
            regulate();            
            led_counter++;
            outLeds.write(led_counter);
            if(angle_reached()){
                s = IDLE;
            }
            break;
        
        case STOP:
            out_ctl_motor_tau.write(0.0);
            led_counter = 0;
            outLeds.write(led_counter);
            break;

        case IDLE:
            regulate();
            led_counter = 15;
            outLeds.write(led_counter);
            break;
            
        default:
            break;
        }
    }
}

void controller::regulate(){
    dt = 0.01;
    //dt = sc_time_stamp().to_seconds() - time;  // Dynamic delta time calculation
    //time = sc_time_stamp().to_seconds();

    q_error_prev = q_error;                         // Save previous error
    q_error = q_target - q;                         // Calculate new error
    q_error_deriv = (q_error - q_error_prev)/dt;    // Calculate the error derivative
    q_error_integ += q_error*dt;                    // Calculate the error integral


    // With gravity compensation:
    // float ctl_gravity_tau = 1.2 * (-9.82) * 0.8 * cos(q);
    // ctl_motor_tau = K_p * q_error + K_d * q_error_deriv - ctl_gravity_tau;

    // PID:
    ctl_motor_tau = K_p * q_error + K_i * q_error_integ + K_d * q_error_deriv;
    out_ctl_motor_tau.write(int(ctl_motor_tau * 1000));
}
