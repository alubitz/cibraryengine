#pragma once

#include "StdAfx.h"

#include "DATKeyframe.h"

namespace DoodAnimTool
{
	class PoseSolverState
	{
		public:
			
			DATKeyframe initial;
			DATKeyframe current;
			DATKeyframe next;

			PoseSolverState(const DATKeyframe& initial);
			~PoseSolverState();

			DATKeyframe GetFinalPose();
	};
}
