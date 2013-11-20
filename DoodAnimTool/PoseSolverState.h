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

			unsigned int* contrib_count;

			float errors[5];

			PoseSolverState(const DATKeyframe& initial);
			~PoseSolverState();

			void PreIteration();
			void PostIteration();

			DATKeyframe GetFinalPose();
	};
}
