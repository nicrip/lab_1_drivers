/******************************************************************************************
* Simple test application to validate the operation of the Pololu software driver
* Sam Claassens
* July 2015
******************************************************************************************/

#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <lcm/lcm.h>
#include <time.h>

// Include the LCM types as well
#include "lcmtypes/njord_servogroup_set_t.h"

using namespace std;

void SendMessage(lcm_t *lcm, int32_t *servoPositions) 
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts); 

	njord_servogroup_set_t msg;
	msg.utime = (long)(ts.tv_nsec / 1000);
	for(int i = 0; i < 6; i++)
		msg.servo_setpoints[i] = servoPositions[i];
	njord_servogroup_set_t_publish(lcm, "NJORD_V2V_VEH1_SET_SERVOGROUP", &msg);
}


int main(int argc, char** argv)
{
 	//
	// Initialize the LCM components
	//
	lcm_t* lcm = lcm_create(NULL);
	if(!lcm) {
		cerr << "Error: Could not initialize LCM." << endl;
		exit(1);
	}	

   cout << "Successfully initialized, going to in the main wait loop..." << endl;

	int32_t setpoints[6] = {3000, 3000, 3000, 3000, 3000, 3000};
	float timet = 0;
	float freq = 1.0f;
	while (1) {
		//lcm_handle(lcm);
		usleep(50000);

		timet += 0.1;

		for(int i = 0; i < 6; i++)
			setpoints[i] = (int32_t)(750.0*sin(2.0*3.141*freq*timet)) + 3000;

		SendMessage(lcm, &setpoints[0]);
		cout << "Iterating..." << endl;
	}

	lcm_destroy(lcm);
}
