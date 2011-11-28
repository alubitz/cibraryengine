#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct ProfilingTimer
	{
		string text;
		unsigned long int start;

		ProfilingTimer(string text);
		~ProfilingTimer();
	};
}
