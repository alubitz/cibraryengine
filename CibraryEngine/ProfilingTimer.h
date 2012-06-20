#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct ProfilingTimer
	{
	private:

		unsigned long int start;
		unsigned long int stop;

	public:

		void Start();
		float Stop();

		float GetElapsedTime();
	};
}
