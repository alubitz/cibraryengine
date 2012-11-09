#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class PhysicsConstraint;
	struct ContentMan;

	struct HardwareAcceleratedComputation;

	class ConstraintGraphSolver
	{
		private:

			HardwareAcceleratedComputation* joint_constraint_hac;
			HardwareAcceleratedComputation* contact_point_dd_hac;			// contact point between two dynamic objects
			HardwareAcceleratedComputation* contact_point_ds_hac;			// contact point between a dynamic object and a static object

			HardwareAcceleratedComputation* vdata_copy_hac;					// copy velocity data between constraint batches

		public:

			ConstraintGraphSolver();
			~ConstraintGraphSolver();

			void Init(ContentMan* content);

			void Solve(unsigned int iterations, vector<PhysicsConstraint*>& constraints);
	};
}
