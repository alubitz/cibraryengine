#pragma once

#include "StdAfx.h"

#include "DATKeyframe.h"
#include "JointOrientations.h"

namespace DoodAnimTool
{
	class PoseSolverState
	{
		public:

			static const unsigned int ERROR_TYPES = 7;
			
			JointOrientations joints;

			DATKeyframe initial;
			DATKeyframe current;
			DATKeyframe next;

			unsigned int* contrib_count;

			float errors[ERROR_TYPES];

			PoseSolverState(const DATKeyframe& initial, const JointOrientations& joints);
			~PoseSolverState();

			void PreIteration();
			void PostIteration();

			DATKeyframe GetFinalPose();
	};
}
