#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	/** Class containing information about a tick of the game clock */
	class TimingInfo
	{
		public:
			/** How much time has elapsed since the last tick */
			double elapsed;
			/** The total amount of time that has passed in the game */
			double total;

			/** Initializes a TimingInfo with elapsed and total both equal to zero */
			TimingInfo();
			/** Initializes a TimingInfo with both values specified */
			TimingInfo(double elapsed, double total);
	};
}
