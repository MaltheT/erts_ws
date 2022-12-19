#include "controller_driver.h"

void controller_driver::test() {

	//Variables
	reset.write(true);
	wait();
	// reset.write(false);
	wait();

	wait();
	wait();

	if (true)
		retval = 0;
	else
		retval = 1;

}
