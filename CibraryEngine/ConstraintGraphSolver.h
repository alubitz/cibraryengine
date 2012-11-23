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

			unsigned int shader_g, shader_v, shader_f;
			unsigned int program;

			unsigned int renderbuffer;
			unsigned int framebuffer;

			// locations of uniform variables in the shader program
			int u_timestep;
			int u_num_rigid_bodies;
			int u_constraint_data;
			int u_velocity_data;
			int u_mass_infos;

			// locations of attribute variables
			int a_constraint_data_index;
			int a_object_indices;

			// vertex buffers
			unsigned int velocity_data_a;
			unsigned int velocity_data_b;
			unsigned int constraint_data;
			unsigned int mass_infos;

			unsigned int v_xfer_indices;

			// buffer textures
			unsigned int vdata_tex_a;
			unsigned int vdata_tex_b;
			unsigned int constraints_tex;
			unsigned int mass_info_tex;

			bool init_ok;

			struct BatchData
			{
				vector<PhysicsConstraint*> constraints;

				/**
				 * Creates a batch containing a subset of the provided constraints containing no adjacent edges
				 * The list is updated to remove the constraints which were put into the created batch
				 */
				BatchData(vector<PhysicsConstraint*>& unassigned);

				void GetVTransferIndices(float* results, boost::unordered_map<RigidBody*, unsigned int>& rb_indices, float*& constraint_data_ptr, unsigned int& constraint_texel_index);
			};

		public:

			ConstraintGraphSolver();
			~ConstraintGraphSolver();

			void Init(ContentMan* content);

			void Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints);
	};
}
