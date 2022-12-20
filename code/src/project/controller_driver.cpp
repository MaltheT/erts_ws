#include "controller_driver.h"

void controller_driver::test() {

	
 	//Variables
	// reset.write(true);
	// std::cout << "Checkpoint 1" << std::endl;

	// std::cout << "Checkpoint 2" << std::endl;
	reset.write(false);
	std::cout << "Checkpoint 3" << std::endl;
	std::cout << "Checkpoint 4" << std::endl;
	out_q_target.write(1.0);
	std::cout << "Q value set" << std::endl;
	wait();
	wait();

	if (true)
		retval = 0;
	else
		retval = -0;

}
