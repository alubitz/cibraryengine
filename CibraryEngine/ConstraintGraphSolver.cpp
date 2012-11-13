#include "StdAfx.h"
#include "ConstraintGraphSolver.h"

#include "Physics.h"
#include "RigidBody.h"

#include "JointConstraint.h"

#include "Serialize.h"

#include "DebugLog.h"
#include "Content.h"

#include "Shader.h"
#include "VertexBuffer.h"
#include "HardwareAcceleratedComputation.h"

#include "TextureBuffer.h"
#include "UniformVariables.h"

namespace CibraryEngine
{
	/*
	 * ConstraintGraphSolver::BatchData methods
	 */
	ConstraintGraphSolver::BatchData::BatchData(vector<PhysicsConstraint*>& unassigned, map<RigidBody*, unsigned int> rb_indices) :
		constraints(),
		v_xfer_indices(NULL),
		v_xfer_tex(NULL),
		eval_obj_indices(NULL)
	{
		vector<PhysicsConstraint*> nu_unassigned;
		SmartHashSet<RigidBody, 37> used_nodes;

		// select which edges will appear in this batch
		for(vector<PhysicsConstraint*>::iterator iter = unassigned.begin(); iter != unassigned.end(); ++iter)
		{
			PhysicsConstraint* constraint = *iter;
			if(used_nodes.Contains(constraint->obj_a) || used_nodes.Contains(constraint->obj_b))
				nu_unassigned.push_back(constraint);
			else
			{
				constraints.push_back(constraint);

				used_nodes.Insert(constraint->obj_a);
				if(constraint->obj_b->MergesSubgraphs())
					used_nodes.Insert(constraint->obj_b);
			}
		}

		// update the reference parameter
		unassigned.assign(nu_unassigned.begin(), nu_unassigned.end());

		// init velocity transfer indices buffer (used as a uniform buffer texture)
		v_xfer_indices = new VertexBuffer();
		v_xfer_indices->AddAttribute("indices", Float, 1);
		v_xfer_indices->SetNumVerts(rb_indices.size());

		float* v_xfer_array = v_xfer_indices->GetFloatPointer("indices");
		memset(v_xfer_array, 0, rb_indices.size() * sizeof(float));
		

		// init constraint eval input index buffer (used as an attribute)
		eval_obj_indices = new VertexBuffer();
		eval_obj_indices->AddAttribute("object_indices", Float, 2);
		eval_obj_indices->SetNumVerts(constraints.size());
		float* eval_obj_indices_ptr = eval_obj_indices->GetFloatPointer("object_indices");

		// populate both of those buffers
		int next_index = 0;
		for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			unsigned int index_a = rb_indices[(*iter)->obj_a];
			unsigned int index_b = rb_indices[(*iter)->obj_b];

			v_xfer_array[index_a] = (float)++next_index;					// odd indices are velocity data from the first object of a constraint pair
			v_xfer_array[index_b] = (float)++next_index;					// even indices are from the second

			*(eval_obj_indices_ptr++) = (float)index_a;
			*(eval_obj_indices_ptr++) = (float)index_b;
		}

		v_xfer_indices->BuildVBO();
		v_xfer_tex = new TextureBuffer(v_xfer_indices, GL_R32F);

		eval_obj_indices->BuildVBO();
	}

	void ConstraintGraphSolver::BatchData::Cleanup()
	{
		if(v_xfer_tex)			{ v_xfer_tex->Dispose();		delete v_xfer_tex;			v_xfer_tex = NULL; }
		if(v_xfer_indices)		{ v_xfer_indices->Dispose();	delete v_xfer_indices;		v_xfer_indices = NULL; }
		if(eval_obj_indices)	{ eval_obj_indices->Dispose();	delete eval_obj_indices;	eval_obj_indices = NULL; }
	}




	/*
	 * ConstraintGraphSolver methods
	 */
	ConstraintGraphSolver::ConstraintGraphSolver() :
		constraint_eval_hac(NULL),
		vdata_copy_hac(NULL),
		constraint_eval_out(NULL),
		velocity_data_a(NULL),
		velocity_data_b(NULL),
		constraint_out_tex(NULL),
		vdata_tex_a(NULL),
		vdata_tex_b(NULL)
	{
	}

	ConstraintGraphSolver::~ConstraintGraphSolver()
	{
		if(constraint_eval_hac)	{ delete constraint_eval_hac;	constraint_eval_hac = NULL; }
		if(vdata_copy_hac)		{ delete vdata_copy_hac;		vdata_copy_hac = NULL; }

		if(constraint_eval_out)	{ constraint_eval_out->Dispose();	delete constraint_eval_out;	constraint_eval_out = NULL; }

		if(constraint_out_tex)	{ constraint_out_tex->Dispose();	delete constraint_out_tex;	constraint_out_tex = NULL; }
		if(vdata_tex_a)			{ vdata_tex_a->Dispose();			delete vdata_tex_a;			vdata_tex_a = NULL; }
		if(vdata_tex_b)			{ vdata_tex_b->Dispose();			delete vdata_tex_b;			vdata_tex_b = NULL; }

		if(velocity_data_a)		{ velocity_data_a->Dispose();		delete velocity_data_a;		velocity_data_a = NULL; }
		if(velocity_data_b)		{ velocity_data_b->Dispose();		delete velocity_data_b;		velocity_data_b = NULL; }
	}

	void ConstraintGraphSolver::Init(ContentMan* content)
	{
		Cache<Shader>* shader_cache = content->GetCache<Shader>();

		// initialize stuff pertaining to constraint evaluator HAC
		constraint_eval_out = new VertexBuffer();
		constraint_eval_out->AddAttribute("vel_a", Float, 4);
		constraint_eval_out->AddAttribute("vel_b", Float, 4);
		constraint_eval_out->AddAttribute("rot_a", Float, 4);
		constraint_eval_out->AddAttribute("rot_b", Float, 4);

		map<string, string> constraint_eval_output_map;
		constraint_eval_output_map["out_vel_a"] = "vel_a";
		constraint_eval_output_map["out_vel_b"] = "vel_b";
		constraint_eval_output_map["out_rot_a"] = "rot_a";
		constraint_eval_output_map["out_rot_b"] = "rot_b";

		constraint_eval_hac = new HardwareAcceleratedComputation(shader_cache->Load("constraint_eval-v"), constraint_eval_output_map, constraint_eval_out);
		ShaderProgram* constraint_eval_prog = constraint_eval_hac->shader_program;

		constraint_eval_prog->AddUniform<int>(				new UniformInt(				"velocity_data_size"				));
		constraint_eval_prog->AddUniform<TextureBuffer>(	new UniformTextureBuffer(	"velocity_data",				2	));

		// TODO: add uniform variables to this shader

		constraint_out_tex = new TextureBuffer(constraint_eval_out, GL_RGBA32F);



		// initialize stuff pertaining to velocity data copy/transfer HAC
		velocity_data_a = new VertexBuffer();
		velocity_data_a->AddAttribute("vel", Float, 4);
		velocity_data_a->AddAttribute("rot", Float, 4);

		map<string, string> vdata_copy_output_map;
		vdata_copy_output_map["out_vel"] = "vel";
		vdata_copy_output_map["out_rot"] = "rot";

		velocity_data_b = VertexBuffer::CreateEmptyCopyAttributes(velocity_data_a);

		vdata_copy_hac = new HardwareAcceleratedComputation(shader_cache->Load("vdata_copy-v"), vdata_copy_output_map, velocity_data_a);
		ShaderProgram* vdata_copy_prog = vdata_copy_hac->shader_program;

		vdata_copy_prog->AddUniform<int>(					new UniformInt(				"constraint_results_size"			));
		vdata_copy_prog->AddUniform<TextureBuffer>(			new UniformTextureBuffer(	"constraint_results",			2	));
		vdata_copy_prog->AddUniform<TextureBuffer>(			new UniformTextureBuffer(	"transfer_indices",				1	));

		vdata_tex_a = new TextureBuffer(velocity_data_a, GL_RGBA32F);
		vdata_tex_b = new TextureBuffer(velocity_data_b, GL_RGBA32F);
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

		// put rigid bodies' velocity data into the vertex buffer
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
			++lv_ptr;

			Vec3 rot = body->GetAngularVelocity();
			*(av_ptr++) = rot.x;
			*(av_ptr++) = rot.y;
			*(av_ptr++) = rot.z;
			++av_ptr;
		}
		velocity_data_a->BuildVBO();

		velocity_data_b->SetNumVerts(num_rigid_bodies);
		velocity_data_b->BuildVBO();

		vdata_tex_a->SetBuffer(velocity_data_a);
		vdata_tex_b->SetBuffer(velocity_data_b);

		vector<BatchData> batches;
		vector<PhysicsConstraint*> unassigned = constraints;
		while(!unassigned.empty())
			batches.push_back(BatchData(unassigned, rb_indices));
		
		Debug(((stringstream&)(stringstream() << "number of batches = " << batches.size() << endl)).str());


		VertexBuffer *active_vdata = velocity_data_a, *inactive_vdata = velocity_data_b;
		TextureBuffer *active_vtex = vdata_tex_a, *inactive_vtex = vdata_tex_b;

		// do the actual solving
		for(unsigned int i = 0; i < iterations; ++i)
			for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			{
				BatchData& batch = *iter;

				// do constraint shader stuff
				ShaderProgram* constraint_eval_prog = constraint_eval_hac->shader_program;
				int velocity_data_size = (int)num_rigid_bodies;
				constraint_eval_prog->SetUniform<int>(				"velocity_data_size",		&velocity_data_size);
				constraint_eval_prog->SetUniform<TextureBuffer>(	"velocity_data",			active_vtex);

				constraint_eval_hac->Process(batch.eval_obj_indices, constraint_eval_out);

				/*
				constraint_eval_out->UpdateDataFromGL();

				Debug("constraint_eval results:\n");

				float* in_v = active_vdata->GetFloatPointer("vel");
				float* indices = batch.eval_obj_indices->GetFloatPointer("object_indices");
				float* out_v = constraint_eval_out->GetFloatPointer("vel_a");
				for(unsigned int i = 0; i < constraint_eval_out->GetNumVerts(); ++i)
				{
					int index = (int)indices[i * 2];
					Vec3 in_vel, out_vel;
					in_vel.x = in_v[index * 4 + 0];
					in_vel.y = in_v[index * 4 + 1];
					in_vel.z = in_v[index * 4 + 2];
					out_vel.x = out_v[i * 4 + 0];
					out_vel.y = out_v[i * 4 + 1];
					out_vel.z = out_v[i * 4 + 2];

					Debug(((stringstream&)(stringstream() << "\tindex = " << index << "; in = (" << in_vel.x << ", " << in_vel.y << ", " << in_vel.z << "); out = (" << out_vel.x << ", " << out_vel.y << ", " << out_vel.z << ")" << endl)).str());
				}
				*/



				// update master array of velocity data
				ShaderProgram* vdata_copy_prog = vdata_copy_hac->shader_program;

				int constraint_results_size = (int)constraint_eval_out->GetNumVerts();
				vdata_copy_prog->SetUniform<int>(					"constraint_results_size",	&constraint_results_size);
				vdata_copy_prog->SetUniform<TextureBuffer>(			"constraint_results",		constraint_out_tex);
				vdata_copy_prog->SetUniform<TextureBuffer>(			"transfer_indices",			batch.v_xfer_tex);

				vdata_copy_hac->Process(active_vdata, inactive_vdata);

				// change which direction the copying is going (back and forth)... can't use one buffer as both input and output or it will be undefined behavior!
				swap(active_vdata, inactive_vdata);
				swap(active_vtex, inactive_vtex);

				//active_vdata->UpdateDataFromGL();
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
			++lv_ptr;

			Vec3 rot;
			rot.x = *(av_ptr++);
			rot.y = *(av_ptr++);
			rot.z = *(av_ptr++);
			++av_ptr;

			body->SetLinearVelocity(vel);
			body->SetAngularVelocity(rot);
		}

		// clean up per-batch stuff
		for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			iter->Cleanup();
	}
}
