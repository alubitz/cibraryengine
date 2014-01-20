#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	class PoseSolverState;
	class DATKeyframe;

	class Constraint
	{
		public:

			virtual ~Constraint() { }

			virtual void InitCachedStuff(PoseSolverState& pose) = 0;
			virtual bool ApplyConstraint(PoseSolverState& pose) = 0;
			virtual void OnAnyChanges(PoseSolverState& pose) = 0;

			virtual float GetErrorAmount(const DATKeyframe& pose) = 0;
	};
}
