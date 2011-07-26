#include "TimingInfo.h"

using namespace CibraryEngine;

TimingInfo::TimingInfo()
{
	elapsed = 0.0;
	total = 0.0;
}

TimingInfo::TimingInfo(double _elapsed, double _total)
{
	elapsed = _elapsed;
	total = _total;
}
