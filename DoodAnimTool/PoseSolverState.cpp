#include "StdAfx.h"
#include "PoseSolverState.h"

#include "DATKeyframe.h"

namespace DoodAnimTool
{
	/*
	 * PoseSolverState methods
	 */
	PoseSolverState::PoseSolverState(const DATKeyframe& initial) : initial(initial), current(initial), next(initial) { }
	PoseSolverState::~PoseSolverState() { }

	DATKeyframe PoseSolverState::GetFinalPose() { return current; }
}
