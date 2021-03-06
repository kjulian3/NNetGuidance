/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

#include <uORB/uORB.h>
#include "math.h"
#include "nnet.hpp"

using namespace aa241x_low;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */

void low_loop(void *nnet)
{
	//Local Static Variables
	//static float way_E[5] = {145.0f,145.0f,130.0f,90.0f,0.0f};
	//static float way_N[5] = {-47.0f,  7.0f, 40.0f,80.0f,0.0f};
	//static float headings[5] ={3.1415926f/2.0f,3.1415926f/2.0f,3.0f*3.1415926f/4.0f,3.0f*3.1415926f/4.0f,0.0f};
	
	static float way_E[15] = {-125.0f, 140.97477f, 109.02523, 140.97477f, 109.02523, 140.97477f,139.72243f, -82.18451, -67.81549f, -82.18451f, -67.81549f, -82.18451f,-92.0f, -88.0f,-56.0f};
	static float way_N[15] = {158.0f, 52.81434f, 41.18566f, 52.81434f, 41.18566f, 52.81434f, 38.5f, -108.40723f, -77.59277f, -108.40723f, -77.59277f, -108.40723f, -93.0f, 185.0f,170.0f};
	static float headings[15] = {0.61087f,-1.22173f,1.91986f,-1.22173f,1.91986f,-1.22173f,-1.8326f,2.70526f,-0.436332f,2.70526f,-0.436332f,2.70526f,2.00713f,0.6109f,-2.00713};
	static int ind = 0;
	//static bool justStarted;
	low_data.ind = ind;

	//Reset every time mission mode begins
	if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) {
		ind = 0;
		low_data.phi_c = 0.0f;
		//low_data.v_c   = ground_speed;
		low_data.lastTime = hrt_absolute_time();
		//justStarted = true;
	}
	//if (justStarted && ((hrt_absolute_time()-low_data.lastTime)>3000000.0f)){
	//	justStarted = false;
	//}
	
	//Only run once about every 0.1 seconds
	if (hrt_absolute_time()-low_data.lastTime > 99000.0f){// && (!justStarted)){
		low_data.lastTime = hrt_absolute_time();

		//Calculate State
		float range_E = way_E[ind]-position_E;
		float range_N = way_N[ind]-position_N;
		float range = sqrt(range_E*range_E+range_N*range_N);

		//Check if need to move to next way point
		if (range < 16.0f) {
			ind = ind+1;

			//Don't allow ind to exceed or equal length of way_E/way_N/headings
			if (ind>14) {
				ind = ind-15;
			}
			range_E = way_E[ind]-position_E;
			range_N = way_N[ind]-position_N;
			range = sqrt(range_E*range_E+range_N*range_N);
		}

		float theta = ((float)atan2(range_N,range_E)) + (ground_course - 3.1415926f/2.0f );
		float bank = -low_data.phi_c;
		float bearing = headings[ind]+(ground_course-3.1415926f/2.0f);

		while (theta > 3.1415926f) {
			theta -= 3.1415926f*2.0f;
		}
		while (theta < -3.1415926f) {
			theta += 3.1415926f*2.0f;
		}

		while (bearing > 3.1415926f) {
			bearing -= 3.1415926f*2.0f;
		}
		while (bearing < -3.1415926f) {
			bearing += 3.1415926f*2.0f;
		}
		//float speed = low_data.v_c;

		//Evaluate Network
		low_data.input1 = range;
		low_data.input2 = theta;
		low_data.input3 = bearing;
		low_data.input4 = bank;
		low_data.input5 = ground_speed;

		float inputs[5] = {range,theta,bearing,bank,ground_speed};
		float outputs[2] = {0.0f,0.0f};

		if (nnet==NULL) {
			low_data.success = -1.0f;
		} else {
			low_data.success = 1.0f;
		}
		evaluate_network(nnet,inputs,outputs);
	
		low_data.output1 = outputs[0];
		low_data.output2 = outputs[1];
		low_data.output3 = -1.0f;//outputs[2];
		low_data.output4 = -1.0f;//outputs[3];
	
	
		//Values to increment for command
		float phiOut = bank;
		//float vOut   = speed;

		//Determine Action Index
		int action = 0;
		float bestValue = -999999999999.0f;
		for (int i=0; i<2;i++){
			if (outputs[i]>bestValue){
				bestValue = outputs[i];
				action = i;
			}
		}
	
		//Set command values
		//float deltaSpeed = 0.0f;
		float deltaPhi   = 0.0f;
		if (action==0){
			deltaPhi = -5.0f*3.1415926f/180.0f;
		} else if (action== 1){
			deltaPhi = 5.0f*3.1415926f/180.0f;
		}
		//Don't let the speed and bank commands exceed their ranges!

		if ((phiOut + deltaPhi) > (72.0f/180.0f*3.1415926f)){
			if (deltaPhi > 0.0f){
				deltaPhi = 72.0f/180.0f*3.1416926f - phiOut;
			}
		}
		if ((phiOut + deltaPhi) < (-72.0f/180.0f*3.1415926f)){
			if (deltaPhi < -0.0f){
				deltaPhi = -72.0f/180.0f*3.1416926f - phiOut;
			}
		}
	
		//Write command values to data structure
		low_data.deltaPhi = deltaPhi;
		//low_data.deltaSpeed=deltaSpeed;
		low_data.phi_c = -(phiOut + deltaPhi);
		//low_data.v_c   = vOut + deltaSpeed;
	}
}
