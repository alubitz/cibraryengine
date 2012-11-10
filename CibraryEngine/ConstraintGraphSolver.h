#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class PhysicsConstraint;
	struct ContentMan;

	struct HardwareAcceleratedComputation;
	struct VertexBuffer;

	class ConstraintGraphSolver
	{
		private:

			HardwareAcceleratedComputation* constraint_eval_hac;
			HardwareAcceleratedComputation* vdata_copy_hac;					// copy velocity data between constraint batches

			VertexBuffer* velocity_data;
			VertexBuffer* constraint_eval_out;

			struct BatchData
			{
				vector<PhysicsConstraint*> constraints;

				vector<unsigned int>		v_xfer_indices;
			};

			void SelectBatches(vector<PhysicsConstraint*>& constraints, vector<BatchData>& batches);

		public:

			ConstraintGraphSolver();
			~ConstraintGraphSolver();

			void Init(ContentMan* content);

			void Solve(unsigned int iterations, vector<PhysicsConstraint*>& constraints);
	};
}
