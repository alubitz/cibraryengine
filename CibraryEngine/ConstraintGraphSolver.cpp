#include "StdAfx.h"
#include "ConstraintGraphSolver.h"

#include "Physics.h"
#include "RigidBody.h"

#include "JointConstraint.h"

#include "DebugLog.h"
#include "Content.h"

#include "Shader.h"
#include "VertexBuffer.h"
#include "HardwareAcceleratedComputation.h"

namespace CibraryEngine
{
	/*
	 * ConstraintGraphSolver methods
	 */
	ConstraintGraphSolver::ConstraintGraphSolver() :
		constraint_eval_hac(NULL),
		vdata_copy_hac(NULL),
		constraint_eval_out(NULL),
		velocity_data_a(NULL),
		velocity_data_b(NULL)
	{
	}

	ConstraintGraphSolver::~ConstraintGraphSolver()
	{
		if(constraint_eval_hac)	{ delete constraint_eval_hac;	constraint_eval_hac = NULL; }
		if(vdata_copy_hac)		{ delete vdata_copy_hac;		vdata_copy_hac = NULL; }

		if(constraint_eval_out)	{ constraint_eval_out->Dispose();	delete constraint_eval_out;	constraint_eval_out = NULL; }
		if(velocity_data_a)		{ velocity_data_a->Dispose();		delete velocity_data_a;		velocity_data_a = NULL; }
		if(velocity_data_b)		{ velocity_data_b->Dispose();		delete velocity_data_b;		velocity_data_b = NULL; }
	}

	void ConstraintGraphSolver::Init(ContentMan* content)
	{
		Cache<Shader>* shader_cache = content->GetCache<Shader>();

		// initialize stuff pertaining to constraint evaluator HAC
		constraint_eval_out = new VertexBuffer(Points);
		constraint_eval_out->AddAttribute("vel_a", Float, 3);
		constraint_eval_out->AddAttribute("vel_b", Float, 3);
		constraint_eval_out->AddAttribute("rot_a", Float, 3);
		constraint_eval_out->AddAttribute("rot_b", Float, 3);

		map<string, string> constraint_eval_output_map;
		constraint_eval_output_map["out_vel_a"] = "vel_a";
		constraint_eval_output_map["out_vel_b"] = "vel_b";
		constraint_eval_output_map["out_rot_a"] = "rot_a";
		constraint_eval_output_map["out_rot_b"] = "rot_b";

		constraint_eval_hac = new HardwareAcceleratedComputation(shader_cache->Load("constraint_eval-v"), constraint_eval_output_map, constraint_eval_out);

		// initialize stuff pertaining to velocity data copy/transfer HAC
		velocity_data_a = new VertexBuffer(Points);
		velocity_data_a->AddAttribute("vel", Float, 3);
		velocity_data_a->AddAttribute("rot", Float, 3);

		map<string, string> vdata_copy_output_map;
		vdata_copy_output_map["out_vel"] = "vel";
		vdata_copy_output_map["out_rot"] = "rot";

		velocity_data_b = VertexBuffer::CreateEmptyCopyAttributes(velocity_data_a);

		vdata_copy_hac = new HardwareAcceleratedComputation(shader_cache->Load("vdata_copy-v"), vdata_copy_output_map, velocity_data_a);
	}


	void ConstraintGraphSolver::SelectBatches(vector<PhysicsConstraint*>& constraints, vector<BatchData>& batches)
	{
		static vector<PhysicsConstraint*> unassigned;
		unassigned.assign(constraints.begin(), constraints.end());

		while(!unassigned.empty())
		{
			static SmartHashSet<RigidBody, 37> used_nodes;
			static vector<PhysicsConstraint*> nu_unassigned;

			static BatchData batch;

			for(vector<PhysicsConstraint*>::iterator iter = unassigned.begin(); iter != unassigned.end(); ++iter)
			{
				PhysicsConstraint* constraint = *iter;
				if(used_nodes.Contains(constraint->obj_a) || used_nodes.Contains(constraint->obj_b))
					nu_unassigned.push_back(constraint);
				else
				{
					batch.constraints.push_back(constraint);

					used_nodes.Insert(constraint->obj_a);
					if(constraint->obj_b->MergesSubgraphs())
						used_nodes.Insert(constraint->obj_b);
				}
			}

			batches.push_back(batch);

			unassigned.assign(nu_unassigned.begin(), nu_unassigned.end());

			batch.constraints.clear();
			used_nodes.Clear();
			nu_unassigned.clear();
		}
	}

	void ConstraintGraphSolver::Solve(unsigned int iterations, vector<PhysicsConstraint*>& constraints)
	{
		// collect unique rigid bodies in a vector, and map to each one its index... should this be non-static only?
		vector<RigidBody*> rigid_bodies;
		map<RigidBody*, unsigned int> rb_indices;

		for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			PhysicsConstraint* constraint = *iter;
			RigidBody *rb_a = constraint->obj_a, *rb_b = constraint->obj_b;

			if(rb_indices.find(rb_a) == rb_indices.end())
			{
				rb_indices[rb_a] = rigid_bodies.size();
				rigid_bodies.push_back(rb_a);
			}

			if(rb_indices.find(rb_b) == rb_indices.end())
			{
				rb_indices[rb_b] = rigid_bodies.size();
				rigid_bodies.push_back(rb_b);
			}
		}

		unsigned int num_rigid_bodies = rigid_bodies.size();

		// put rigid bodies' velocity data into a vertex buffer
		velocity_data_a->SetNumVerts(num_rigid_bodies);
		float* lv_ptr = velocity_data_a->GetFloatPointer("vel");
		float* av_ptr = velocity_data_a->GetFloatPointer("rot");
		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(), rb_end = rigid_bodies.end(); iter != rb_end; ++iter)
		{
			RigidBody* body = *iter;

			Vec3 vel = body->GetLinearVelocity();
			*(lv_ptr++) = vel.x;
			*(lv_ptr++) = vel.y;
			*(lv_ptr++) = vel.z;

			Vec3 rot = body->GetAngularVelocity();
			*(av_ptr++) = rot.x;
			*(av_ptr++) = rot.y;
			*(av_ptr++) = rot.z;
		}
		velocity_data_a->BuildVBO();


		// collect constraints into batches containing no adjacent edges
		static vector<BatchData> batches;

		SelectBatches(constraints, batches);
		Debug(((stringstream&)(stringstream() << "number of batches = " << batches.size() << endl)).str());



		// populate velocity transfer indices (for updating velocities of all rigid bodies given indices into a batch's outputs)
		for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
		{
			BatchData& batch = *iter;

			batch.v_xfer_indices.resize(num_rigid_bodies, 0);

			int next_index = 1;
			for(vector<PhysicsConstraint*>::iterator jter = batch.constraints.begin(); jter != batch.constraints.end(); ++jter)
			{
				batch.v_xfer_indices[rb_indices[(*jter)->obj_a]] = next_index++;
				batch.v_xfer_indices[rb_indices[(*jter)->obj_b]] = next_index++;
			}
		}

		VertexBuffer *active_vdata = velocity_data_a, *inactive_vdata = velocity_data_b;

		// do the actual solving
		for(unsigned int i = 0; i < iterations; ++i)
			for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			{
				BatchData& batch = *iter;

				//constraint_eval_hac->Process(velocity_data, constraint_eval_out);

				vdata_copy_hac->Process(active_vdata, inactive_vdata);
				swap(active_vdata, inactive_vdata);
			}

		// copy linear and angular velocity data from vertex buffer back to the corresponding RigidBody objects
		active_vdata->UpdateDataFromGL();
		lv_ptr = active_vdata->GetFloatPointer("vel");
		av_ptr = active_vdata->GetFloatPointer("rot");

		for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
		{
			RigidBody* body = rigid_bodies[i];

			Vec3 vel;
			vel.x = *(lv_ptr++);
			vel.y = *(lv_ptr++);
			vel.z = *(lv_ptr++);

			Vec3 rot;
			rot.x = *(av_ptr++);
			rot.y = *(av_ptr++);
			rot.z = *(av_ptr++);

			body->SetLinearVelocity(vel);
			body->SetAngularVelocity(rot);
		}

		batches.clear();
	}
}
