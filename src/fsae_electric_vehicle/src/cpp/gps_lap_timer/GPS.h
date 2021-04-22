#pragma once

//#include "gps_timer.h"   // Our header
#include <cstdint>
#include <iostream>
#include <cassert>

// MSVC
#define float double

#define MIN(x, y)                     (((x) < (y)) ? (x) : (y))
#define MAX(x, y)                     (((x) > (y)) ? (x) : (y))
#define DEGTORAD(deg)                 (deg*57.29577995f)
#define D_TO_RADIANS                  (PI/180.0f)

// GPS RMC sentence fields.
#define RMC_PREFIX                    0
#define RMC_TIME                      1
#define RMC_STATUS                    2
#define RMC_LATITUDE                  3
#define RMC_LATITUDE_PREFIX           4
#define RMC_LONGITUDE                 5
#define RMC_LONGITUDE_PREFIX          6
#define RMC_SPEED                     7
#define RMC_TRACK                     8
#define RMC_DATE                      9
#define RMC_MAGNETIC_VARIATION        10
#define RMC_MAGNETIC_VARIATION_PREFIX 11
#define RMC_FAA_MODE_INDICATOR        12
#define RMC_CHECKSUM                  13

#define LATITUDE                      0x01
#define LONGITUDE                     0x02

struct point_t { float x, y; };
struct line_t { point_t p0, p1; };

static float EstablishStartLine(char *[]);
static void Run(float, char *[]);
static float Distance(const point_t, const point_t);
static void IntersectPoint(const point_t, const point_t, point_t*);
static bool LineIntersection(const line_t);
static void StartLine(const float, const float, const float);
static bool GetRMCSentence(char* []);
static float atof_(char []);
static char* strtok_(char*, const char*);
static char hex(const char);
static float ConvertToSeconds(char*);
static bool Equal(float, float);
static bool Within30(const uint16_t , const uint16_t);
static void GeoCopy(const char*, char*, const unsigned char);
static void Prepend(char*, const char*);
static size_t ParseRMC(char*, char* []);
static bool Checksum(char*);
static void DisplayTime(const uint8_t, const float);






// GPS update frequency and period.
const std::size_t GPS_UPDATE_FREQUENCY = 5; // Hz.
static constexpr float GPS_UPDATE_PERIOD{ 1.0f/GPS_UPDATE_FREQUENCY };

// Distance calculations.
static constexpr float PI{ 3.141592654 };
static constexpr float EARTH_RADIUS{ 3956.0 }; // In miles. 

// Definitions for the startline.
static constexpr float LINE_WIDTH{ 50.0f };
static constexpr float LINE_WIDTH_2{ 25.0f };
static constexpr float PROJECTION_DISTANCE{ 100.0f };

// Maximum possible characters in a GPS string (+ fudge).
const std::size_t GPS_STRING_LENGTH = 80;



// Lap time class.
struct lap
{
	void setStart(const float t)
	{
		assert(t >= 0.0f);
		start = t;
	}

	void setStop(const float t)
	{ 
		assert(t >= 0.0f);
		stop = t;
	}

	float getStart() const { return start; }
	float getStop() const { return stop; }
	
	float getTime() const 
	{
		if (stop < start)
			return 0.0f;

		return (stop - start);
	}

private:
	static float toTimeStamp(float seconds)
	{
		uint16_t hours = (uint16_t)(seconds / 3600);
		uint16_t minutes = (uint16_t)(seconds / 60 - (hours * 60));
		float fs = seconds - (uint16_t)(seconds / 100) * 100;
		float timestamp = (((hours * 100) + minutes) * 100) + fs;

		return timestamp;
	}

	static float toSeconds(float time)
	{
		uint16_t hm = (uint16_t)(time / 100);
		uint16_t hours = (uint16_t)(time / 10000);
		uint16_t minutes = (hm - (hours * 100) + (hours * 60));
		float seconds = time - (hm * 100) + (minutes * 60);

		return seconds;
	}

	float start;
	float stop;
};



// Error descriptions.
static const char* description[] = { "NO ERROR",  "CHECKSUM FAILURE", "INVALID SENTENCE", "NO FIX", "NON-SEQUENTIAL SENTENCE", "END OF FILE" };

// Simplistic error tracker.
struct err
{
public:
	enum ID : size_t { NONE = 0, CHECKSUM, BAD_SENTENCE, NO_FIX, TIME_STAMP, FILE_EOF };

	void Clear() { error_ = ID::NONE; }
	void SetError(const ID id) { error_ = id; }
	ID GetError() const { return error_; }
	const char* GetDescription() const { return description[error_]; }

private:
	ID error_;
};


//#define FILE_INPUT

#ifdef FILE_INPUT
const char* filePath = "/home/btc54/Desktop/formulaEmbedded/src/fsae_electric_vehicle/src/cpp/gps_lap_timer/data.txt";
FILE* file = NULL;
bool FILE_INPUT_EOF = false;
#endif