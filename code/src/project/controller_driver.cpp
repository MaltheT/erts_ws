#include "controller_driver.h"

void controller_driver::test() {

	reset.write(false);

	wait();
	wait();

	if (true)
		retval = 0;
	else
		retval = -0;

}
