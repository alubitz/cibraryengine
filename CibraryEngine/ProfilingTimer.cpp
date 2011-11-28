#include "StdAfx.h"
#include "ProfilingTimer.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * ProfilingTimer methods
	 */

	ProfilingTimer::ProfilingTimer(string text) : text(text)
	{
		LARGE_INTEGER t;
		QueryPerformanceCounter(&t); 

		start = (unsigned long int)t.QuadPart;
	}

	ProfilingTimer::~ProfilingTimer()
	{
		LARGE_INTEGER t;
		QueryPerformanceCounter(&t);

		unsigned long int end = (unsigned long int)t.QuadPart;

		QueryPerformanceFrequency(&t);

		unsigned long int freq = (unsigned long int)t.QuadPart;

		float elapsed = float(end - start) / freq;

		stringstream ss;
		ss << "Operation \"" << text << "\" took " << elapsed << " seconds" << endl;
		Debug(ss.str());
	}
}
