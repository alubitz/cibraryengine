#include "StdAfx.h"
#include "TimingInfo.h"

namespace CibraryEngine
{
	/*
	 * TimingInfo methods
	 */
	TimingInfo::TimingInfo() : elapsed(0.0), total(0.0) { }
	TimingInfo::TimingInfo(float elapsed, float total) : elapsed(elapsed), total(total) { }
}