#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class PhysicsConstraint;
	class RigidBody;

	struct ContentMan;

	struct HardwareAcceleratedComputation;
	struct VertexBuffer;
	class TextureBuffer;

	class ConstraintGraphSolver
	{
		private:

			HardwareAcceleratedComputation* constraint_eval_hac;
			HardwareAcceleratedComputation* vdata_copy_hac;					// copy velocity data between constraint batches

			VertexBuffer* constraint_eval_out;
			VertexBuffer* velocity_data_a;
			VertexBuffer* velocity_data_b;
			VertexBuffer* constraint_data;
			VertexBuffer* mass_infos;

			TextureBuffer* constraint_out_tex;
			TextureBuffer* vdata_tex_a;
			TextureBuffer* vdata_tex_b;
			TextureBuffer* constraints_tex;
			TextureBuffer* mass_info_tex;

			bool init_ok;

			struct BatchData
			{
				vector<PhysicsConstraint*> constraints;

				VertexBuffer* v_xfer_indices;
				TextureBuffer* v_xfer_tex;

				VertexBuffer* eval_obj_indices;

				/**
				 * Creates a batch containing a subset of the provided constraints containing no adjacent edges
				 * The list is updated to remove the constraints which were put into the created batch
				 */
				BatchData(vector<PhysicsConstraint*>& unassigned, boost::unordered_map<RigidBody*, unsigned int>& rb_indices, float*& constraint_data_ptr, unsigned int& constraint_texel_index);

				void Cleanup();
			};

		public:

			ConstraintGraphSolver();
			~ConstraintGraphSolver();

			void Init(ContentMan* content);

			void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints);
	};
}
