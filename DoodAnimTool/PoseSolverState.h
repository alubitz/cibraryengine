#pragma once

#include "StdAfx.h"

#include "DATKeyframe.h"

namespace DoodAnimTool
{
	class PoseSolverState
	{
		public:

			static const unsigned int ERROR_TYPES = 7;
			
			DATKeyframe initial;
			DATKeyframe current;
			DATKeyframe next;

			unsigned int* contrib_count;

			float errors[ERROR_TYPES];

			PoseSolverState(const DATKeyframe& initial);
			~PoseSolverState();

			void PreIteration();
			void PostIteration();

			DATKeyframe GetFinalPose();
	};
}
