#include "StdAfx.h"
#include "ProfilingTimer.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * ProfilingTimer methods
	 */
	void ProfilingTimer::Start()
	{
		LARGE_INTEGER t;
		QueryPerformanceCounter(&t); 

		_start = (unsigned long int)t.QuadPart;
	}

	float ProfilingTimer::Stop()
	{
		LARGE_INTEGER t;
		QueryPerformanceCounter(&t);

		_stop = (unsigned long int)t.QuadPart;

		return GetElapsedTime();
	}

	float ProfilingTimer::GetElapsedTime()
	{
		LARGE_INTEGER t;
		QueryPerformanceFrequency(&t);

		unsigned long int freq = (unsigned long int)t.QuadPart;
		return float(_stop - _start) / freq;
	}
}
