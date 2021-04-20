#include <iostream>
#include <cstdio>
#include <cctype>
#include <cassert>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include "gps_timer.h"
//#include "GPS.h"



template<typename T> // Prototype exists because templates dont exist in C
static T htoi(const char* hexStr);

/* #ifdef __cplusplus
extern "C" {
#endif */

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

/* #ifdef __cplusplus
}
#endif
 */


template<typename T>
static T htoi(const char* hexStr) {
	T value = T{ 0 };

	if (hexStr != nullptr)
		for (size_t i = 0; i < sizeof(T) * 2; ++i)
			value |= hex(tolower(hexStr[i])) << (8 * sizeof(T) - 4 * (i + 1));

	return value;
};