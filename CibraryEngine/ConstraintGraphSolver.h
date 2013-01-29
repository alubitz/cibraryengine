#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class PhysicsConstraint;
	
	struct ContentMan;

	class ConstraintGraphSolver
	{
		

		public:

			virtual ~ConstraintGraphSolver() { }

			virtual void Init(ContentMan* content) { };

			virtual void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints) = 0;
	};
}
