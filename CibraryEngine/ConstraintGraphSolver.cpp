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
		joint_constraint_hac(NULL),
		contact_point_dd_hac(NULL),
		contact_point_ds_hac(NULL),
		vdata_copy_hac(NULL)
	{
	}

	ConstraintGraphSolver::~ConstraintGraphSolver()
	{
		if(joint_constraint_hac)	{ delete joint_constraint_hac;	joint_constraint_hac = NULL; }
		if(contact_point_dd_hac)	{ delete contact_point_dd_hac;	contact_point_dd_hac = NULL; }
		if(contact_point_ds_hac)	{ delete contact_point_ds_hac;	contact_point_ds_hac = NULL; }
		if(vdata_copy_hac)			{ delete vdata_copy_hac;		vdata_copy_hac = NULL; }
	}

	void ConstraintGraphSolver::Init(ContentMan* content)
	{
		Cache<Shader>* shader_cache = content->GetCache<Shader>();

		VertexBuffer* constraint_shader_dd_proto = new VertexBuffer(Points);			// output prototype for constraints with two dynamic objects
		constraint_shader_dd_proto->AddAttribute("out_vel_a", Float, 3);
		constraint_shader_dd_proto->AddAttribute("out_vel_b", Float, 3);
		constraint_shader_dd_proto->AddAttribute("out_rot_a", Float, 3);
		constraint_shader_dd_proto->AddAttribute("out_rot_b", Float, 3);

		joint_constraint_hac = new HardwareAcceleratedComputation(	shader_cache->Load("joint_constraint-v"),	constraint_shader_dd_proto);
		contact_point_dd_hac = new HardwareAcceleratedComputation(	shader_cache->Load("contact_point_dd-v"),	constraint_shader_dd_proto);

		VertexBuffer* constraint_shader_ds_proto = new VertexBuffer(Points);			// output prototype for constraints with one dynamic object
		constraint_shader_ds_proto->AddAttribute("out_vel_a", Float, 3);
		constraint_shader_ds_proto->AddAttribute("out_rot_a", Float, 3);

		contact_point_ds_hac = new HardwareAcceleratedComputation(	shader_cache->Load("contact_point_ds-v"),	constraint_shader_ds_proto);

		VertexBuffer* vdata_copy_proto = new VertexBuffer(Points);						// output prototype for velocity data copier
		vdata_copy_proto->AddAttribute("out_vel", Float, 3);
		vdata_copy_proto->AddAttribute("out_rot", Float, 3);

		vdata_copy_hac = new HardwareAcceleratedComputation(		shader_cache->Load("vdata_copy-v"),			vdata_copy_proto);

		// clean up output prototypes
		constraint_shader_dd_proto->Dispose();	delete constraint_shader_dd_proto;
		constraint_shader_ds_proto->Dispose();	delete constraint_shader_ds_proto;
		vdata_copy_proto->Dispose();			delete vdata_copy_proto;
	}

	void ConstraintGraphSolver::Solve(unsigned int iterations, vector<PhysicsConstraint*>& constraints)
	{
		vector<RigidBody*> rigid_bodies;
		map<RigidBody*, unsigned int> rb_indices;

		struct BatchData
		{
			vector<ContactPoint*>		dd_contact_points;
			vector<ContactPoint*>		ds_contact_points;
			vector<JointConstraint*>	joint_constraints;

			vector<unsigned int>		v_xfer_indices;
		};
		static vector<BatchData> batches;

		static vector<PhysicsConstraint*> unassigned;
		unassigned.assign(constraints.begin(), constraints.end());

		// collect constraints into batches containing no adjacent edges
		// collect unique non-static rigid bodies in a vector, and map to each one its index
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
					// classify the constraint according to which HAC it will use
					if(ContactPoint* cp = dynamic_cast<ContactPoint*>(constraint))
					{
						if(cp->obj_b->MergesSubgraphs())
							batch.dd_contact_points.push_back(cp);
						else
							batch.ds_contact_points.push_back(cp);
					}
					else
					{
						assert(constraint->obj_b->MergesSubgraphs());
						batch.joint_constraints.push_back((JointConstraint*)constraint);
					}

					RigidBody *rb_a = constraint->obj_a, *rb_b = constraint->obj_b;

					if(rb_indices.find(rb_a) == rb_indices.end())
					{
						rb_indices[rb_a] = rigid_bodies.size();
						rigid_bodies.push_back(rb_a);
					}
					used_nodes.Insert(rb_a);

					if(rb_b->MergesSubgraphs())
					{
						if(rb_indices.find(rb_b) == rb_indices.end())
						{
							rb_indices[rb_b] = rigid_bodies.size();
							rigid_bodies.push_back(rb_b);
						}

						used_nodes.Insert(rb_b);
					}
				}
			}

			unassigned.assign(nu_unassigned.begin(), nu_unassigned.end());

			batches.push_back(batch);

			batch.dd_contact_points.clear();
			batch.ds_contact_points.clear();
			batch.joint_constraints.clear();
			used_nodes.Clear();
			nu_unassigned.clear();
		}

		unsigned int num_rigid_bodies = rigid_bodies.size();

		// put rigid bodies' velocity data into arrays
		VertexBuffer* velocity_data = new VertexBuffer(Points);
		velocity_data->AddAttribute("vel", Float, 3);
		velocity_data->AddAttribute("rot", Float, 3);
		velocity_data->SetNumVerts(num_rigid_bodies);
		
		float *lv_ptr = velocity_data->GetFloatPointer("vel"), *av_ptr = velocity_data->GetFloatPointer("rot");
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



		// populate velocity transfer indices (for updating velocities given indices into a batch's outputs)
		for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
		{
			BatchData& batch = *iter;
			
			batch.v_xfer_indices.resize(num_rigid_bodies, 0);

			unsigned int next_index = 1;
			for(vector<ContactPoint*>::iterator jter = batch.dd_contact_points.begin(); jter != batch.dd_contact_points.end(); ++jter)
			{
				batch.v_xfer_indices[rb_indices[(*jter)->obj_a]] = next_index++;
				batch.v_xfer_indices[rb_indices[(*jter)->obj_b]] = next_index++;
			}

			for(vector<ContactPoint*>::iterator jter = batch.ds_contact_points.begin(); jter != batch.ds_contact_points.end(); ++jter)
			{
				batch.v_xfer_indices[rb_indices[(*jter)->obj_a]] = next_index++;
				++next_index;
			}

			for(vector<JointConstraint*>::iterator jter = batch.joint_constraints.begin(); jter != batch.joint_constraints.end(); ++jter)
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

				for(vector<ContactPoint*>::iterator jter = batch.dd_contact_points.begin(); jter != batch.dd_contact_points.end(); ++jter)
					(*jter)->DoConstraintAction();

				for(vector<ContactPoint*>::iterator jter = batch.ds_contact_points.begin(); jter != batch.ds_contact_points.end(); ++jter)
					(*jter)->DoConstraintAction();

				for(vector<JointConstraint*>::iterator jter = batch.joint_constraints.begin(); jter != batch.joint_constraints.end(); ++jter)
					(*jter)->DoConstraintAction();
			}

		// copy data from linear and angular velocity arrays back to the corresponding RigidBody objects
		for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
		{
			RigidBody* body = rigid_bodies[i];

			if(!body->MergesSubgraphs())
				Debug("Oops, this RigidBody can't move but it's been put in the rigid_bodies vector anyway!\n");

			// TODO: implement this for real
		}

		velocity_data->Dispose();
		delete velocity_data;

		batches.clear();
	}
}
