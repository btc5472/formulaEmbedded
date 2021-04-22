#include "ros/ros.h"
#include <fsae_electric_vehicle/gps.h> /* This gps.h is actually referencing 'gps.msg' file in msg folder.
Idk why its like this but it wont compile without it */
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <cstdint>
#include <cassert>
#include <math.h>
#include <array>
#include "CANController.h"
//#include "gps_timer.h"	// Custom header
#include "GPS.h"			// Custom header

#define _CRT_SECURE_NO_WARNINGS // For MSVC

// Coordinate of start/finish location.
point_t startPoint;
// Startline endpoints.
line_t startingLine;
// Heading crossing start/finish.
uint16_t startHeading;
// Coordinates of current & previous GPS location.
line_t track;

// File input buffer.
char buffer[GPS_STRING_LENGTH];

// GPS errors.
err error;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_lap_timer");
	ros::NodeHandle n;
  	ros::Publisher gps_lap_timer_pub = n.advertise<fsae_electric_vehicle::gps>("gps_lap_timer", 1000);
  	fsae_electric_vehicle::gps gps_lap_timer; // constructor

 	char* gpsTokens[RMC_CHECKSUM + 1]; // Pointer to GPS RMC (string) fields

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
	/********** Maybe replace this while loop with a function that waits to save cpu cycles***********/
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
		error.SetError(err::ID::BAD_SENTENCE);
		ROS_FATAL("Cannot establish startline due to GetRMCSentence\n");
		ROS_DEBUG("Cannot establish startline due to GetRMCSentence\n");
		exit(error.GetError());
	}

	ros::Rate loop_rate(30); // This means that loop rate can be up to 30 times per second

  	while (ros::ok()) {
		ROS_INFO_ONCE("ROS is ok!");
		if (GetRMCSentence(gpsTokens)) { // Receive GPS data from file or from CANBUS
			if (ts != 0.0f) {
				Run(ts, gpsTokens);
			}
		} else {
			error.SetError(err::ID::BAD_SENTENCE);
			ROS_FATAL_THROTTLE(1, "RMC sentence was lost!"); // Prints error description once per second
			ROS_DEBUG_THROTTLE(1, "RMC sentence was lost!");
			exit(error.GetError());
		}

		// Store GPS data 
		gps_lap_timer.time = atof(gpsTokens[1]);
		gps_lap_timer.latitude = atof(gpsTokens[4]);
		gps_lap_timer.longitude = atof(gpsTokens[6]);
		gps_lap_timer.speed = atof(gpsTokens[7]);
		gps_lap_timer.heading = atof(gpsTokens[8]);
		gps_lap_timer.magneticVariation = atof(gpsTokens[11]);
   		gps_lap_timer_pub.publish(gps_lap_timer);

    	ros::spinOnce();
   		loop_rate.sleep();
  	}

#ifdef FILE_INPUT // Close file
	if (file)
		fclose(file);
#endif

}



// An RMC sentence is a standardized string of chars emitted by GPS units
static bool GetRMCSentence(char* tokens[]) {
	error.Clear();

#ifdef FILE_INPUT
	std::cout << "before fgets func";
	if (fgets(buffer, GPS_STRING_LENGTH, file) == NULL) {
		std::cout << "Cannot read file into buffer. Func = GetRMCSentence(). File = gps.h----\n";
	}
	//if (fgets(buffer, GPS_STRING_LENGTH, file) == NULL)
	//{
		//std::cout << "In fgets";
		//error.SetError(err::ID::FILE_EOF);
		//return false;
	//}
#else
	float seconds = 0, latitude = 0, longitude = 0, speed = 0, magneticVar = 0, trueCourse = 0;
	/*********************
	 * So I think the reason why can.start & can.getData are not defined is because CANController.cpp isnt being compiled into a .o file and therefore cannot be linked. To fix this I think I may have to change my CMake file
	 * ******************/
	CANController can; // Start the CABUS header on the Jetson/Quasar board
	can.start("can0");
	auto data = can.getData(0x34, 0x1FFFFFFF); // First param is idFilter
	if (data.has_value()) {
		//std::memcpy(&buffer, data->data, RMC_CHECKSUM + 1); //
		seconds = data->data[0]; // 'seconds' can be later converted into hours:minutes:seconds later if needed
		latitude = data->data[1];
		longitude = data->data[2];
		speed = data->data[3];
		magneticVar = data->data[4];
		trueCourse= data->data[5];
	} else {
		return false;
	}
	
	//Apply conversions below and store to buffer
	
	
	
	
#endif

	// RMC sentence?
	if (strncmp("$GPRMC", buffer, 6) == 0)
	{
		// Terminate sentence at eol.
		char* eol = strchr(buffer, 0x0a);
		if (eol != NULL)
#ifdef FILE_INPUT
			buffer[eol - buffer] = 0;
#else
			buffer[eol - buffer - 1] = 0;
#endif

		// Confirm crc.
		if (!Checksum(buffer))
		{
			error.SetError(err::ID::CHECKSUM);
			ROS_FATAL("CRC not confirmed!");
			ROS_DEBUG("CRC not confirmed!");
			return false;
		}

		// Parse the GPS RMC string and check for valid fix.
		if (ParseRMC(buffer, tokens) == RMC_CHECKSUM) {
			if (tokens[RMC_STATUS] == nullptr || *tokens[RMC_STATUS] != 'A') {
				error.SetError(err::ID::NO_FIX);
				ROS_DEBUG("GPS has no fix");
				return false;
			}
			else
				return true;
		}
	}

	error.SetError(err::ID::BAD_SENTENCE);
	ROS_DEBUG("Not an RMC sentence!");
	return false;
}



static float EstablishStartLine(char *tokens[]) {
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



static void Run(float timeStamp, char *tokens[]) {
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

	std::cout << clock[++ticToc & 0x01] << '\r';

// Previous position GPS time stamp.
	float prevTimeStamp = timeStamp;

	// Confirm sentence is sequential.
	timeStamp = ConvertToSeconds(tokens[RMC_TIME]);
	if (!Equal(timeStamp, prevTimeStamp + GPS_UPDATE_PERIOD)) {
		error.SetError(err::ID::TIME_STAMP);
		ROS_DEBUG("GPS has no fix");
	}

	// Get current track position (lat, long).
	if (tokens[RMC_LATITUDE] != nullptr || tokens[RMC_LONGITUDE] != nullptr) {
		char temp[12];

		GeoCopy(tokens[RMC_LATITUDE], temp, LATITUDE);
		track.p1.x = atof_(temp);
		GeoCopy(tokens[RMC_LONGITUDE], temp, LONGITUDE);
		track.p1.y = atof_(temp);
	}
	else


	// Ignore gps sentences for 1 second after crossing start/finish.
	if (hzCounter < GPS_UPDATE_FREQUENCY) {
		hzCounter++;

		// Prepare for next iteration.
		track.p0.x = track.p1.x;
		track.p0.y = track.p1.y;
	}
	
	// Heading sanity check & check if crossed start/finish line?
	if (Within30(startHeading, (uint16_t)atol(tokens[RMC_TRACK])) && LineIntersection(track)) {
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
		if (numLaps == 0 || lapData[numLaps].getTime() < bestTime.second) {
			// Announce new fast lap.
			std::cout << " << Fast Lap";
			bestTime = std::make_pair(numLaps, lapData[numLaps].getTime());
		}
		std::cout << "\n";

		// Increment counters. These probably dont work
		numLaps++;
		hzCounter = 1;
	}

	// Prepare for next iteration.
	track.p0.x = track.p1.x;
	track.p0.y = track.p1.y;
}



static float Distance(const point_t t1, const point_t t2) {
	float Lat1, Long1, Lat2, Long2;		// Coordinates in degrees.
	float dlat, dlon;			// Change in location.
	float a, d;
	float c;				// Great Circle distance (radians).

	Lat1 = (float)((uint32_t)(t1.y / 100.0f));
	Lat1 = (float)(Lat1 + (t1.y - Lat1 * 100.0f) / 60.0f);
	Lat1 = (float)(Lat1 * D_TO_RADIANS);

	Long1 = (float)((uint32_t)(t1.x / 100.0f));
	Long1 = (float)(Long1 + (t1.x - Long1 * 100.0f) / 60.0f);
	Long1 = (float)(Long1 * D_TO_RADIANS);

	Lat2 = (float)((uint32_t)(t2.y / 100.0f));
	Lat2 = (float)(Lat2 + (t2.y - Lat2 * 100.0f) / 60.0f);
	Lat2 = (float)(Lat2 * D_TO_RADIANS);

	Long2 = (float)((uint32_t)(t2.x / 100.0f));
	Long2 = (float)(Long2 + (t2.x - Long2 * 100.0f) / 60.0f);
	Long2 = (float)(Long2 * D_TO_RADIANS);

	dlat = Lat2 - Lat1;
	dlon = Long2 - Long1;

	a = (float)((sin(dlat / 2.0f) * sin(dlat / 2.0f)) + cos(Lat1) * cos(Lat2) * (sin(dlon / 2.0f) * sin(dlon / 2.0f)));
	c = (float)(2.0f * atan2(sqrt(a), sqrt(1.0f - a)));

	d = (float)(EARTH_RADIUS * c);

	return d;
}



// Construct a startline.
static void StartLine(const float sx, const float sy, const float shdg) {
	float tx, ty; // Projected track coordinates.
	float m, b;   // Slope & y intercept.

	// Project racetrack along current heading.
	tx = sx + PROJECTION_DISTANCE * cos(DEGTORAD(shdg));
	ty = sy + PROJECTION_DISTANCE * sin(DEGTORAD(shdg));
	// Projected racetrack slope & y-intercept. 
	m = (sy - ty) / (sx - tx);
	b = sy - (m * sx);

	// Construct perpendicular (startline) slope & y-intercept.
	m = -1.0f / m;
	b = sy - (m * sx);

	// Define endpoints of the perpendicular.
	tx = sx + LINE_WIDTH_2; // Note: tx re-used as a temporary value here.
	startingLine.p0.y = (m * tx + b);
	startingLine.p0.x = tx;
	tx -= LINE_WIDTH;
	startingLine.p1.y = (m * tx + b);
	startingLine.p1.x = tx;
}



// 2d line intersection.
static bool LineIntersection(const line_t track) {
	float z;
	int16_t s1, s2, s3, s4;

	// Quick rejection test.
	if (!(MAX(startingLine.p0.x, startingLine.p1.x) >= MIN(track.p0.x, track.p1.x) &&
		MAX(track.p0.x, track.p1.x) >= MIN(startingLine.p0.x, startingLine.p1.x) &&
		MAX(startingLine.p0.y, startingLine.p1.y) >= MIN(track.p0.y, track.p1.y) &&
		MAX(track.p0.y, track.p1.y) >= MIN(startingLine.p0.y, startingLine.p1.y)))
		return false;

	// Straddle tests.
	if ((z = ((track.p0.x - startingLine.p0.x) * (startingLine.p1.y - startingLine.p0.y)) - ((track.p0.y - startingLine.p0.y) * (startingLine.p1.x - startingLine.p0.x))) < 0.0f)
		s1 = -1; // Counterclockwise. 
	else if (z > 0.0f)
		s1 = 1;  // Clockwise.
	else
		s1 = 0;  // Collinear.

	if ((z = ((track.p1.x - startingLine.p0.x) * (startingLine.p1.y - startingLine.p0.y)) - ((track.p1.y - startingLine.p0.y) * (startingLine.p1.x - startingLine.p0.x))) < 0.0f)
		s2 = -1;
	else if (z > 0.0f)
		s2 = 1;
	else
		s2 = 0;

	if ((z = ((startingLine.p0.x - track.p0.x) * (track.p1.y - track.p0.y)) - ((startingLine.p0.y - track.p0.y) * (track.p1.x - track.p0.x))) < 0.0f)
		s3 = -1;
	else if (z > 0.0f)
		s3 = 1;
	else
		s3 = 0;

	if ((z = ((startingLine.p1.x - track.p0.x) * (track.p1.y - track.p0.y)) - ((startingLine.p1.y - track.p0.y) * (track.p1.x - track.p0.x))) < 0.0f)
		s4 = -1;
	else if (z > 0.0f)
		s4 = 1;
	else
		s4 = 0;

	if ((s1 * s2 <= 0) && (s3 * s4 <= 0))
		return true;

	// Line segments do not intersect.
	return false;
}



// 2d lines point of intersection.
static void IntersectPoint(const point_t p1, const point_t p2, point_t* i) {
	float denom, numera, mua; //numerb, mub;

	// No checks because it is assumed the lines intersect.
	denom = (startingLine.p1.y - startingLine.p0.y) * (p2.x - p1.x) - (startingLine.p1.x - startingLine.p0.x) * (p2.y - p1.y);
	numera = (startingLine.p1.x - startingLine.p0.x) * (p1.y - startingLine.p0.y) - (startingLine.p1.y - startingLine.p0.y) * (p1.x - startingLine.p0.x);
	//numerb = (p2.x - p1.x)*(p1.y - startingLine1.y) - (p2.y - p1.y)*(p1.x - startingLine1.x);

	mua = numera / denom;
	//mub = numerb/denom;
	i->x = p1.x + mua * (p2.x - p1.x);
	i->y = p1.y + mua * (p2.y - p1.y);
}


/*************************************************** GPS Utility Functions *********************************************/
static void DisplayTime(const uint8_t, const float);

// A very basic atof function (no exponentials, no sign).
static float atof_(char s[]) {
	float val, power;
	int8_t i = 0;

	while (!isdigit(s[i]))
		i++;

	for (val = 0.0f; isdigit(s[i]); i++)
		val = 10.0f * val + (s[i] - '0');

	if (s[i] == '.')
		i++;

	for (power = 1.0f; isdigit(s[i]); i++)
	{
		val = 10.0f * val + (s[i] - '0');
		power *= 10.0f;
	}

	return val / power;
}



// strtok implementation which recognizes consecutive delimiters.
static char* strtok_(char* str, const char* delim) {
	static char* staticStr = 0;          // Stores last address.
	int i = 0, strLen = 0, delimLen = 0; // Indexes.

	// If delimiter is NULL or no more chars remaining.
	if (delim == 0 || (str == 0 && staticStr == 0))
		return 0;

	if (str == 0)
		str = staticStr;

	// Get length of string and delimiter.
	while (str[strLen])
		strLen++;
	while (delim[delimLen])
		delimLen++;

	// Find a delimiter.
	char* p = strpbrk(str, delim);
	if (p)
		i = p - str;
	else
	{
		// If no delimiters, return str.
		staticStr = 0;
		return str;
	}

	// Terminate the string.
	str[i] = '\0';

	// Save remaining string.
	if ((str + i + 1) != 0)
		staticStr = (str + i + 1);
	else
		staticStr = 0;

	return str;
}



// Convert hex string to decimal.
static char hex(const char ch) {
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	if (ch >= 'a' && ch <= 'f')
		return ch - 'a' + 10;

	return 0;
}



static float ConvertToSeconds(char* time) {
	if (time == nullptr)
		return 0.0f;

	float ft = atof_(time);
	
	uint16_t hm = (uint16_t)(ft / 100);
	uint16_t hours = (uint16_t)(ft / 10000);
	uint16_t minutes = (hm - (hours * 100) + (hours * 60));
	float seconds = ft - (hm * 100) + (minutes * 60);

	return seconds;
}



// Determine if floats are relatively equal.
static bool Equal(float a, float b) { return fabs(a - b) <= FLT_EPSILON; }



// Check heading and angle within 30 degrees.
static bool Within30(const uint16_t a, const uint16_t h) { return ((360 - abs(a - h) % 360 < 30) || (abs(a - h) % 360 < 30)); }



// Copy lat/long strings and format as ddd.dddd
static void GeoCopy(const char* s, char* d, const unsigned char value) {
	assert(s != nullptr && d != nullptr);

	int i = 0;

	// Copy all numerals, insert/skip decimal point.
	do {
		if (value == LONGITUDE && i == 3)
		{
			*d++ = '.';
			i++;
		}

		if (value == LATITUDE && i == 2)
		{
			*d++ = '.';
			i++;
		}

		if ((*s >= '0') && (*s <= '9'))
		{
			*d++ = *s;
			i++;
		}
	} while (*s++ != '\0');

	// Null terminate.
	*d = '\0';
}



// Prepends s onto d. Assumes d has enough space allocated for the combined string.
static void Prepend(char* d, const char* s) {
	assert(s != nullptr && d != nullptr); 
	
	size_t len = strlen(s);

	memmove(d + len, d, strlen(d) + 1);
	memcpy(d, s, len);
}



// Parse GPS string into tokens
static size_t ParseRMC(char* sentence, char* tokens[]) {
	assert(sentence != nullptr);

	size_t n = 0;

	tokens[n] = strtok_(sentence, ",");
	while (tokens[n] && n < RMC_CHECKSUM)
		tokens[++n] = strtok_(NULL, ",*");

	return n;
}



template<typename T>
static T htoi(const char* hexStr) {
	T value = T{ 0 };

	if (hexStr != nullptr)
		for (size_t i = 0; i < sizeof(T) * 2; ++i)
			value |= hex(tolower(hexStr[i])) << (8 * sizeof(T) - 4 * (i + 1));

	return value;
};



// Verify the checksum of the RMC string
static bool Checksum(char* sentence)
{
	assert(sentence != nullptr);

	uint8_t crc = 0;
	uint8_t n = htoi<uint8_t>(&sentence[strlen(sentence) - 2]);

	// Skip initial '$' and '*' + last 2 bytes (crc).
	for (size_t i = 1; i < strlen(sentence) - 3; i++)
		crc ^= sentence[i];

	return (crc == n);
}



// Convert float seconds to MM::SS.SS format.
static void DisplayTime(const uint8_t n, const float ft) {
	assert(ft > 0.);

	char s1[16];

	memset(&s1[10], 0, 6);
	uint16_t m = (uint16_t)ft / 60;
	float fs = ft - (m * 60);
	sprintf(s1, "%.02d:%05.2f ", m, fs);

	if (n) { // Prepend lap number.
		char s2[6];
		sprintf(s2, "%d: ", n);
		Prepend(s1, s2);
	}

	std::cout << s1;
}


/***************************************************** TODO **********************************************************************/
// Should be able to start & end racing sessions
// Reading data from CANBUS doesnt work. Thats in gps.h in GetRMCSentence()
// Test and debug this program while connected to Jetson and CANBUS
// Vehicle data should be stored locally on the Quasar if there is no connection to the server





/************************************************ Unused code ****************************************************
// UBX Protocol.
//UBX   SYNC  CLSS  ID    LENGTH     MSG                            CHECKSUM
//0xB5, 0x62, 0x06, 0x08, 0x06,0x00, 0x64,0x00,0x01,0x00,0x01,0x00, 0x7A,0x12 // 10Hz
//0xB5, 0x62, 0x06, 0x08, 0x06,0x00, 0xC8,0x00,0x01,0x00,0x01,0x00, 0xDE,0x6A // 5Hz
//0xB5, 0x62, 0x06, 0x08, 0x06,0x00, 0xE8,0x03,0x01,0x00,0x01,0x00, 0x01,0x39 // 1Hz
// Can also disable/enable NMEA sentences with a text command: 
// $PUBX,40,GLL,1,0,0,0,0,0*5D enables the GLL sentence on the serial port.
// $PUBX,41,1,0007,0003,19200,0*25 sets the GPS modules baud rate to 19200.
/*std::array<unsigned char, 14> gpsUpdateRate = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A }; // 5Hz update rate.
std::array<unsigned char, 16> gpsDisableGGA = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24 }; // GxGGA off
std::array<unsigned char, 16> gpsDisableGGL = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B }; // GxGLL off
std::array<unsigned char, 16> gpsDisableGSA = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 }; // GxGSA off
std::array<unsigned char, 16> gpsDisableGSV = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 }; // GxGSV off
std::array<unsigned char, 16> gpsDisableVTG = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 }; // GxVTG off
std::array<unsigned char, 32> gpsBaud19200 = { "$PUBX,41,1,0007,0003,19200,0*25" }; // 19200 baud rate*/

/*static void gpsSetup(const PORT p)
{
	assert(p != NULL);

	SendData(p, &gpsDisableGGA[0], gpsDisableGGA.size()); // Disable GGA.
	SendData(p, &gpsDisableGGL[0], gpsDisableGGL.size()); // Disable GGL.
	SendData(p, &gpsDisableGSA[0], gpsDisableGSA.size()); // DIsable GSA.
	SendData(p, &gpsDisableGSV[0], gpsDisableGSV.size()); // Disable GSV.
	SendData(p, &gpsDisableVTG[0], gpsDisableVTG.size()); // Disable VTG.
	SendData(p, &gpsUpdateRate[0], gpsUpdateRate.size()); // Set update rate at 5Hz.
	//SendData(p, &gpsBaud19200[0], gpsBaud19200.size()); // Set 19200 baud rate.
}*/