#pragma once

#include "StdAfx.h"
#include "ConstraintGraphSolver.h"

#include "SmartHashSet.h"

#include "Physics.h"

namespace CibraryEngine
{
	using namespace std;

	class CPUConstraintGraphSolver : public ConstraintGraphSolver
	{
		public:

			void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints);
	};
}
