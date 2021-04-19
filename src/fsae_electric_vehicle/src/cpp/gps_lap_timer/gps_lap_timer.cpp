#include "ros/ros.h"
#include "CANController.h"
#include <fsae_electric_vehicle/gps.h> // This gps.h is actually referencing the gps.msg file in the msg folder. I dont know why its like this but it wont compile without it
#include "gps_timer.h"   // Our header
#include "utility.h"     // Utility functions
#include "gps.h"         // GPS specific functions
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <cstdint>
#include <cassert>
#include <math.h>
#include <array>

#define _CRT_SECURE_NO_WARNINGS

//Use ROS_INFO for printf-style logging, and ROS_INFO_STREAM for cout-style logging

static void Prepend(char*, const char*);
static void DisplayTime(const uint8_t, const float);
static float EstablishStartLine(char *tokens[]);
static void Run(float timeStamp, char *tokens[]);

int main(int argc, char **argv)
{
	//std::cout << "Starting gps_lap_timer" << std::endl;
	ros::init(argc, argv, "gps_lap_timer");
  	//std::cout << "initialized gps_lap_timer node" << std::endl;

	ros::NodeHandle n;
 	//std::cout << "After node handle calling ros::start()" << std::endl;

  	ros::Publisher gps_lap_timer_pub = n.advertise<fsae_electric_vehicle::gps>("gps_lap_timer", 1000);
 	//std::cout << "After gps_lap_timer_pub" << std::endl;

  	fsae_electric_vehicle::gps gps_lap_timer; // constructor
  	//std::cout << "listening gps_lap_timer" << std::endl;

 	char* gpsTokens[RMC_CHECKSUM + 1]; // Pointer to gps rmc (string) fields

#ifdef FILE_INPUT
	// Attempt to open gps data file
	file = fopen(filePath, "r");
	std::cout << "after open file";
	if (file == NULL) {
		printf("-----------ERROR OPENING FILE-----------\n");
		ROS_INFO("-----------ERROR OPENING FILE 2-----------\n");
		exit(-1);
	}
	//fgets(buffer, GPS_STRING_LENGTH, file);
	//std::cout << buffer[0];
#else
  	CANController can; // Start the CABUS header on the Jetson/Quasar board
	can.start("can0");
	// Wait for GPS fix
	do {
		// Nothing
	} while (!GetRMCSentence(gpsTokens));
	std::cout << "\nGPS status active!";
#endif
	
	// Establish StartLine
	float ts;
	if (GetRMCSentence(gpsTokens))
		ts = EstablishStartLine(gpsTokens);
	else {
		std::cout << "Cannot establish startline due to GetRMCSentence\n";
		ROS_INFO("ROS_INFO Cannot establish startline due to GetRMCSentence\n");
		exit(-2);
	}

	ros::Rate loop_rate(30); // Loop rate can be up to 30 times per second

  	while (ros::ok()) {
		std::cout << "\nROS is ok!";
		if (GetRMCSentence(gpsTokens)) { // Receive gps data from file or from CANBUS
			//std::cout << std::endl; // Add code here to pause StartLine generation until button push
			if (ts != 0.0f) {
				Run(ts, gpsTokens);
				//break;
			}
		}
		gps_lap_timer.time = atof(gpsTokens[1]);
		gps_lap_timer.latitude = atof(gpsTokens[4]);
		gps_lap_timer.longitude = atof(gpsTokens[6]);
		gps_lap_timer.speed = atof(gpsTokens[7]);
		gps_lap_timer.heading = atof(gpsTokens[8]);
		gps_lap_timer.magneticVariation = atof(gpsTokens[11]);
   		gps_lap_timer_pub.publish(gps_lap_timer);
    	ros::spinOnce();
   		loop_rate.sleep();
  	} // while

#ifdef FILE_INPUT // Close file
	if (file)
		fclose(file);
#endif

} // main







// Prepends s onto d. Assumes d has enough space allocated for the combined string.
static void Prepend(char* d, const char* s)
{
	assert(s != nullptr && d != nullptr); 
	
	size_t len = strlen(s);

	memmove(d + len, d, strlen(d) + 1);
	memcpy(d, s, len);
}

// Convert float seconds to MM::SS.SS format.
static void DisplayTime(const uint8_t n, const float ft) 
{
	assert(ft > 0.);

	char s1[16];

	memset(&s1[10], 0, 6);
	uint16_t m = (uint16_t)ft / 60;
	float fs = ft - (m * 60);
	sprintf(s1, "%.02d:%05.2f ", m, fs);

	if (n) // Prepend lap number.
	{
		char s2[6];
		sprintf(s2, "%d: ", n);
		Prepend(s1, s2);
	}

	std::cout << s1;
}

static float EstablishStartLine(char *tokens[])
{
	float ts;

#ifdef FILE_INPUT
	// Safeway parking lot: $GPRMC,194924.80,A,3203.02116,N,11042.41425,W,1.304,30.95,120120,,,A*48
	startPoint.x = (float)32.0302116;
	startPoint.y = (float)110.4241425;
	// Heading while crossing start/finish.
	startHeading = 31;
	// Position timestamp.
	char t[] = "194924.80";
	ts = ConvertToSeconds(t);
#else
	if (tokens[RMC_TIME] == nullptr || tokens[RMC_TRACK] == nullptr ||
	    tokens[RMC_LATITUDE] == nullptr || tokens[RMC_LONGITUDE] == nullptr)
		return 0.0f;

	// Position timestamp.
	ts = ConvertToSeconds(tokens[RMC_TIME]);

	// Get current track position (lat, long).
	char temp[12];
	GeoCopy(tokens[RMC_LATITUDE], temp, LATITUDE);
	startPoint.x = atof_(temp);
	GeoCopy(tokens[RMC_LONGITUDE], temp, LONGITUDE);
	startPoint.y = atof_(temp);

	// Heading while crossing start/finish.
	startHeading = atoi(tokens[RMC_TRACK]);
#endif

	// Define startline.
	StartLine((float)startPoint.x, (float)startPoint.y, (float)startHeading);
	track.p0.x = startPoint.x;
	track.p0.y = startPoint.y;

	return ts;
}

static void Run(float timeStamp, char *tokens[])
{
	// Lap counters.
	uint8_t numLaps = 0;
	uint16_t hzCounter = 1;
	// Lap data.
	std::array<lap, 256> lapData;
	// Best lap time (lap #, time).
	std::pair<uint8_t, float> bestTime(0, 0.0f);

	uint8_t ticToc = 0;
	unsigned char clock[2] = { 47, 92 };

	// Note timestamp of startline point.
	lapData[numLaps].setStart(timeStamp);

	// Main gps string processing loop.
	//while (1)
	//{
		if (!GetRMCSentence(tokens))
		{

#ifdef FILE_INPUT
			if (error.GetError() == err::ID::FILE_EOF)
				return;
#endif
			std::cout << error.GetDescription() << std::endl;
			//continue;
		}
		else
			std::cout << clock[++ticToc & 0x01] << '\r';

		// Previous position GPS time stamp.
		float prevTimeStamp = timeStamp;

		// Confirm sentence is sequential.
		timeStamp = ConvertToSeconds(tokens[RMC_TIME]);
		if (!Equal(timeStamp, prevTimeStamp + GPS_UPDATE_PERIOD))
		{
			error.SetError(err::ID::TIME_STAMP);
			std::cout << error.GetDescription() << std::endl;
			//continue;
		}

		// Get current track position (lat, long).
		if (tokens[RMC_LATITUDE] != nullptr || tokens[RMC_LONGITUDE] != nullptr)
		{
			char temp[12];

			GeoCopy(tokens[RMC_LATITUDE], temp, LATITUDE);
			track.p1.x = atof_(temp);
			GeoCopy(tokens[RMC_LONGITUDE], temp, LONGITUDE);
			track.p1.y = atof_(temp);
		}
		else
			//continue;

		// Ignore gps sentences for 1 second after crossing start/finish.
		if (hzCounter < GPS_UPDATE_FREQUENCY)
		{
			hzCounter++;

			// Prepare for next iteration.
			track.p0.x = track.p1.x;
			track.p0.y = track.p1.y;
			//continue;
		}
		
		// Heading sanity check & check if crossed start/finish line?
		if (Within30(startHeading, (uint16_t)atol(tokens[RMC_TRACK])) && LineIntersection(track))
		{
			point_t intersectPoint;

			// Calculate track/start line intersection point.
			IntersectPoint(track.p0, track.p1, &intersectPoint);

			// Overall length of this track segment.
			float totDist = Distance(track.p0, track.p1);
			// Length from start line intersection point to track segment end point.
			float segDist = Distance(intersectPoint, track.p1);

			// Calculate startline crossing time for this and next lap.
			float xTime = timeStamp - (GPS_UPDATE_PERIOD * (segDist / totDist));
			lapData[numLaps].setStop(xTime);
			lapData[numLaps + 1].setStart(xTime);

			// Determine current lap stats.
			DisplayTime(numLaps + 1, lapData[numLaps].getTime());
			if (numLaps > 0)
				DisplayTime(bestTime.first + 1, bestTime.second);

			// Is this lap a new best?
			if (numLaps == 0 || lapData[numLaps].getTime() < bestTime.second)
			{
				// Announce new fast lap.
				std::cout << " << Fast Lap";
				bestTime = std::make_pair(numLaps, lapData[numLaps].getTime());
			}
			std::cout << "\n";

			// Increment counters.
			numLaps++;
			hzCounter = 1;
		}

		// Prepare for next iteration.
		track.p0.x = track.p1.x;
		track.p0.y = track.p1.y;
	//}
}

/***************************************************** TODO **********************************************************************/
// Should be able to start & end racing sessions
// Reading data from CANBUS doesnt work. Thats in gps.h in GetRMCSentence()
// Test and debug this program while connected to Jetson and CANBUS
// Vehicle data should be stored locally on the Quasar if there is no connection to the server