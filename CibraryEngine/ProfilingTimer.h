#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct ProfilingTimer
	{
	private:

		unsigned long int _start;
		unsigned long int _stop;

	public:

		void Start();
		float Stop();
		float GetAndRestart() { float result = Stop(); _start = _stop; return result; }

		float GetElapsedTime();
	};
}
