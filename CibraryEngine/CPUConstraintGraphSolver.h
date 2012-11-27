#pragma once

#include "StdAfx.h"
#include "ConstraintGraphSolver.h"

#include "Physics.h"

namespace CibraryEngine
{
	using namespace std;

	class CPUConstraintGraphSolver : public ConstraintGraphSolver
	{
		public:

			void Init(ContentMan* content) { }

			void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints)
			{
				for(unsigned int i = 0; i < iterations; ++i)
					for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
						(*iter)->DoConstraintAction();
			}
	};
}
