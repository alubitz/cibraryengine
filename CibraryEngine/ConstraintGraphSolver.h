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

			// formerly HardwareAcceleratedComputation* constraint_eval_hac
			unsigned int shader, program;

			// locations of uniform variables in the shader program
			int u_timestep;
			int u_num_rigid_bodies;
			int u_constraint_data;
			int u_velocity_data;
			int u_mass_infos;

			// locations of attribute variables
			int a_constraint_data_index;
			int a_object_indices;

			// formerly VertexBuffer*
			unsigned int velocity_data_a;
			unsigned int velocity_data_b;
			unsigned int constraint_data;
			unsigned int mass_infos;

			// formerly TextureBuffer*
			unsigned int vdata_tex_a;
			unsigned int vdata_tex_b;
			unsigned int constraints_tex;
			unsigned int mass_info_tex;

			bool init_ok;

			struct BatchData
			{
				vector<PhysicsConstraint*> constraints;

				// formerly VertexBuffer*
				unsigned int v_xfer_indices;

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
