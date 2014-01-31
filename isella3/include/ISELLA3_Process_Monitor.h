/************************************************************************
Fraunhofer IPA, 2011
ISELLA3 - ArmModule 

Author: Attila Achenbach, email: attila.achenbach@googlemail.com
************************************************************************/

// ISELLA3 Process Monitor.h : header for the ISELLA3 Process Monitor
//
// This file defines the object ProcessMonitor which provides a set of
// functions for measuring and monitoring process values.
// '-> Measure
//     '-> Time
//     '-> Time difference
//     '-> CurrentMust
//     '-> CurrentIs
//     '-> AngleMust (Alpha, Beta, Alpha_0, Alpha_1)
//     '-> AngleIs (Alpha, Beta, Alpha_0, Alpha_1)
// '-> and print to console or file

#pragma once

// Includes added by AA
//#include "stdafx.h"
#include <sys/time.h>
#include <string>
#include <sstream>
#include <fstream> // for writing to files
#include <iomanip> // for "setprecision" (of float values)
using namespace std;

// Defines by AA
//#ifndef PROCESSDATA
//	#define PROCESSDATA "ISELLA3_Process_Data.txt" // Define filename for monitoring
//#endif


//
//
// Process Monitor

class CISELLA3ProcessMonitor
{
public:
	// Constructor
	CISELLA3ProcessMonitor() :
		sstrBuffer(std::string())
		{
			T_Start.tv_usec = 0;
			T_Stop.tv_usec = 0;
			T_Stamp.tv_usec = 0;
		}

	unsigned long * ulCurrentTime();
	unsigned long * ulStart();
	unsigned long * ulStop();
	unsigned long * ulDiff();
	unsigned long * ulTimestamp();
	unsigned long * ulDiffTimestamp();
	void ClearBuffer();
	void WriteProcessDataToBuffer(string strValueTag, bool * bValue);
	void WriteProcessDataToBuffer(string strValueTag, short * sValue);
	void WriteProcessDataToBuffer(string strValueTag, int * iValue);
	void WriteProcessDataToBuffer(string strValueTag, double * dValue);
	void WriteProcessDataToBuffer(string strValueTag, long * lValue);
	void WriteProcessDataToBuffer(string strValueTag, unsigned long * ulValue);
	void WriteProcessDataToBuffer(string strValueTag, unsigned long ulValue);
	void WriteProcessDataToBuffer(string strValueTag, float * fValue);
	void WriteProcessDataToBuffer(string strValueTag, char * cValue);
	void WriteProcessDataToBuffer(string strValueTag, string * strValue);
	void WriteBufferEndl();
	void WriteBufferToFile();
	void WriteBufferToFile(string strFilename);
	void MaxonRecorder();

//	LARGE_INTEGER liT_Start; // Move back to private: after sine generator experiment
private:
	stringstream sstrBuffer;
	timeval T_Start;
	unsigned long ulT_Start;
	timeval T_Stop;
	unsigned long ulT_Stop;
	unsigned long ulT_Diff;
	timeval T_Stamp;
	unsigned long ulT_Stamp;
	ofstream processdatafile;
};
