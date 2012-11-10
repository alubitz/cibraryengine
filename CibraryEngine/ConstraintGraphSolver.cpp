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
		velocity_data(NULL),
		constraint_eval_out(NULL)
	{
	}

	ConstraintGraphSolver::~ConstraintGraphSolver()
	{
		if(constraint_eval_hac)	{ delete constraint_eval_hac;	constraint_eval_hac = NULL; }
		if(vdata_copy_hac)		{ delete vdata_copy_hac;		vdata_copy_hac = NULL; }

		if(velocity_data)		{ velocity_data->Dispose();			delete velocity_data;		velocity_data = NULL; }
		if(constraint_eval_out)	{ constraint_eval_out->Dispose();	delete constraint_eval_out;	constraint_eval_out = NULL; }
	}

	void ConstraintGraphSolver::Init(ContentMan* content)
	{
		Cache<Shader>* shader_cache = content->GetCache<Shader>();

		constraint_eval_out = new VertexBuffer(Points);
		constraint_eval_out->AddAttribute(	"out_vel_a",	Float, 3);
		constraint_eval_out->AddAttribute(	"out_vel_b",	Float, 3);
		constraint_eval_out->AddAttribute(	"out_rot_a",	Float, 3);
		constraint_eval_out->AddAttribute(	"out_rot_b",	Float, 3);

		velocity_data = new VertexBuffer(Points);
		velocity_data->AddAttribute(		"vel",			Float, 3);
		velocity_data->AddAttribute(		"rot",			Float, 3);

		constraint_eval_hac = new HardwareAcceleratedComputation(	shader_cache->Load("constraint_eval-v"),	constraint_eval_out);
		vdata_copy_hac = new HardwareAcceleratedComputation(		shader_cache->Load("vdata_copy-v"),			velocity_data);		
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
			batch.constraints.clear();

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

			unassigned.assign(nu_unassigned.begin(), nu_unassigned.end());

			batches.push_back(batch);

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
		velocity_data->SetNumVerts(num_rigid_bodies);
		float* lv_ptr = velocity_data->GetFloatPointer("vel");
		float* av_ptr = velocity_data->GetFloatPointer("rot");
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
		velocity_data->BuildVBO();


		// collect constraints into batches containing no adjacent edges
		static vector<BatchData> batches;

		SelectBatches(constraints, batches);
		Debug(((stringstream&)(stringstream() << "batches.size() = " << batches.size() << endl)).str());



		// populate velocity transfer indices (for updating velocities of all rigid bodies given indices into a batch's outputs)
		for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
		{
			BatchData& batch = *iter;
			
			batch.v_xfer_indices.resize(num_rigid_bodies, 0);

			unsigned int next_index = 1;
			for(vector<PhysicsConstraint*>::iterator jter = batch.constraints.begin(); jter != batch.constraints.end(); ++jter)
			{
				batch.v_xfer_indices[rb_indices[(*jter)->obj_a]] = next_index++;
				batch.v_xfer_indices[rb_indices[(*jter)->obj_b]] = next_index++;
			}
		}


		// do the actual solving
		for(unsigned int i = 0; i < iterations; ++i)
			for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			{
				BatchData& batch = *iter;

				//constraint_eval_hac->Process(velocity_data, constraint_eval_out);
				//vdata_copy_hac->Process(velocity_data, velocity_data);
			}

		// copy linear and angular velocity data from vertex buffer back to the corresponding RigidBody objects
		velocity_data->UpdateDataFromGL();
		lv_ptr = velocity_data->GetFloatPointer("vel");
		av_ptr = velocity_data->GetFloatPointer("rot");

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
