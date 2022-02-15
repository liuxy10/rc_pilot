#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <controller.h>
#include <rc/math.h>
#include <rc/time.h>
#include <settings.h>
#include <state_machine.h>

#include <LQR.h>


/*
For Charan: Put your matrices here with this example format. 
The names should be: hover, rollRight, rollLeft, pitchF, pitchB

static double hover[][6] = {
    {K11, 0.0, 0.0, 0.0, 0.0, 0.0}, 
    {0.0, K22, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, K33, 0.0, 0.0, 0.0}, 
    {0.0, 0.0, 0.0, K44, 0.0, 0.0}, 
    {0.0, 0.0, 0.0, 0.0, K55, 0.0}, 
    {0.0, 0.0, 0.0, 0.0, 0.0, K66}};

*/






double* dists[5]; 

static double* optGain[][6]; 

int __nearestSubspace(void){
	double dist = -1; 
	int idx = -1; 

	// Hover, R+, R-, P+, P-
	
	dists[0] = sqrt(pow(state_estimate.roll, 2) + pow(state_estimate.pitch, 2)); 
	dists[1] = sqrt(pow(state_estimate.roll - TENDEGS, 2) + pow(state_estimate.pitch, 2)); 
	dists[2] = sqrt(pow(state_estimate.roll + TENDEGS, 2) + pow(state_estimate.pitch, 2)); 
	dists[3] = sqrt(pow(state_estimate.roll, 2) + pow(state_estimate.pitch - TENDEGS, 2)); 
	dists[4] = sqrt(pow(state_estimate.roll, 2) + pow(state_estimate.pitch + TENDEGS, 2)); 

	for (int i = 0; i < 5; i++){
		if (dists[i] > dist){
			dist = dists[i]; 
			idx = i; 
		}
	}
	return idx; 
}

int __optimalGain(void){
	int optKey = __nearestSubspace(); 

	if (optKey < 0){
		fprintf(stderr, "ERROR: nearest subspace not found\n");
		return -1; 
	}

	switch (optKey){

		case 0: 
			optGain = hover; 
		case 1: 
			optGain = rollRight;
		case 2: 
			optGain = rollLeft;
		case 3: 
			optGain = pitchF;
		case 4: 
			optGain = pitchB;
	}

	return 0; 


}

int runLQR(setpoint_t* setpoint){

	__nearestSubspace(); 

	double* state[6] = {state_estimate.roll, state_estimate.roll_dot, ...
	 state_estimate.pitch, state_estimate.pitch_dot, state_estimate.yaw, state_estimate.yaw_dot};

	for (int i = 0; i < 6; i++){
		setpoint.roll += optGain[0][i]*state[i]; 
		setpoint.pitch += optGain[2][i]*state[i]; 
		setpoint.yaw += optGain[4][i]*state[i]; 
		setpoint.roll_dot += optGain[1][i]*state[i]; 
		setpoint.pitch_dot += optGain[3][i]*state[i]; 
		setpoint.yaw_dot += optGain[5][i]*state[i]; 
	}

	return 0; 

}

















