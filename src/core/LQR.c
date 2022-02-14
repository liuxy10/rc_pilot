#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <controller.h>
#include <rc/math.h>
#include <rc/time.h>
#include <settings.h>
#include <state_machine.h>

#include <LQR.h>

double* dists[5]; 

static double* optGain[][8]; 

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

int __optimalGain(){
	int optKey = __nearestSubspace(); 

	if (optKey < 0)
		fprintf(stderr, "ERROR: nearest subspace not found\n");
}

