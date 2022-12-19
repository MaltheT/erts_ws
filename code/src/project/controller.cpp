#include "controller.h"

void controller::step()
{
    wait();
    while(1)
    {
        wait();
        q = in_snr_q.read();
        
        //dt = current_t - last_t;                      // Dynamic time elapsed
        q_error_prev = q_error;                         // Save previous error
        q_error = q_target - q;                         // Calculate new error
        q_error_deriv = (q_error - q_error_prev)*dt;    // Calculate the error derivative
        q_error_integ += q_error*dt;                    // Calculate the error integral

        // PID:
        ctl_motor_tau = K_p * q_error + K_i * q_error_integ + K_d * q_error_deriv;
        out_ctl_motor_tau.write(ctl_motor_tau);

        // Step:  
        // out_ctl_motor_tau.write(1);
    }
}