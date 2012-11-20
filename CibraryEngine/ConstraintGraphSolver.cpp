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

#include "ProfilingTimer.h"

#define TEXELS_PER_JC 9
#define TEXELS_PER_CP 6

#define PROFILE_CGRAPH 1


namespace CibraryEngine
{
#if PROFILE_CGRAPH
	static float timer_collect_and_count = 0.0f;
	static float timer_vdata_init = 0.0f;
	static float timer_make_batches = 0.0f;
	static float timer_batch_vbos = 0.0f;
	static float timer_process = 0.0f;
	static float timer_retrieve = 0.0f;
	static float timer_total = 0.0f;
	static unsigned int counter_cgraph = 0;
#endif



	/*
	 * ConstraintGraphSolver::BatchData methods
	 */
	ConstraintGraphSolver::BatchData::BatchData(vector<PhysicsConstraint*>& unassigned, boost::unordered_map<RigidBody*, unsigned int>& rb_indices, float*& data_ptr, unsigned int& texel_index) :
		constraints(),
		v_xfer_indices(NULL)
	{
		static vector<PhysicsConstraint*> nu_unassigned;
		static SmartHashSet<RigidBody, 37> used_nodes;

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

		nu_unassigned.clear();
		used_nodes.Clear();

		// init velocity transfer indices buffer (used as a uniform buffer texture)
		v_xfer_indices = new VertexBuffer();
		v_xfer_indices->AddAttribute("constraint_data_index", Float, 1);
		v_xfer_indices->AddAttribute("object_indices", Float, 2);
		v_xfer_indices->SetNumVerts(rb_indices.size());

		float* constraint_index_array = v_xfer_indices->GetFloatPointer("constraint_data_index");
		memset(constraint_index_array, 0, rb_indices.size() * sizeof(float));

		float* object_index_array = v_xfer_indices->GetFloatPointer("object_indices");

		// populate both of those buffers
		int next_index = 0;
		for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			unsigned int index_a = rb_indices[(*iter)->obj_a];
			unsigned int index_b = rb_indices[(*iter)->obj_b];

			constraint_index_array[index_a] = constraint_index_array[index_b] = (float)(texel_index + 1);

			object_index_array[index_a * 2    ] = object_index_array[index_b * 2    ] = (float)index_a;
			object_index_array[index_a * 2 + 1] = object_index_array[index_b * 2 + 1] = (float)index_b;

			if(JointConstraint* jc = dynamic_cast<JointConstraint*>(*iter))
			{
				jc->WriteDataToBuffer(data_ptr);

				data_ptr += TEXELS_PER_JC * 4;
				texel_index += TEXELS_PER_JC;
			}
			else
			{
				((ContactPoint*)*iter)->WriteDataToBuffer(data_ptr);

				data_ptr += TEXELS_PER_CP * 4;
				texel_index += TEXELS_PER_CP;
			}
		}
	}

	void ConstraintGraphSolver::BatchData::Cleanup() { if(v_xfer_indices) { v_xfer_indices->Dispose(); delete v_xfer_indices; v_xfer_indices = NULL; } }




	/*
	 * ConstraintGraphSolver methods
	 */
	ConstraintGraphSolver::ConstraintGraphSolver() :
		constraint_eval_hac(NULL),
		velocity_data_a(NULL),
		velocity_data_b(NULL),
		constraint_data(NULL),
		mass_infos(NULL),
		vdata_tex_a(NULL),
		vdata_tex_b(NULL),
		constraints_tex(NULL),
		mass_info_tex(NULL),
		init_ok(false)
	{
	}

	ConstraintGraphSolver::~ConstraintGraphSolver()
	{
		if(constraint_eval_hac)	{ delete constraint_eval_hac;	constraint_eval_hac = NULL;	}

		if(vdata_tex_a)			{ vdata_tex_a->Dispose();			delete vdata_tex_a;			vdata_tex_a = NULL;			}
		if(vdata_tex_b)			{ vdata_tex_b->Dispose();			delete vdata_tex_b;			vdata_tex_b = NULL;			}
		if(constraints_tex)		{ constraints_tex->Dispose();		delete constraints_tex;		constraints_tex = NULL;		}
		if(mass_info_tex)		{ mass_info_tex->Dispose();			delete mass_info_tex;		mass_info_tex = NULL;		}

		if(velocity_data_a)		{ velocity_data_a->Dispose();		delete velocity_data_a;		velocity_data_a = NULL;		}
		if(velocity_data_b)		{ velocity_data_b->Dispose();		delete velocity_data_b;		velocity_data_b = NULL;		}
		if(constraint_data)		{ constraint_data->Dispose();		delete constraint_data;		constraint_data = NULL;		}
		if(mass_infos)			{ mass_infos->Dispose();			delete mass_infos;			mass_infos = NULL;			}

#if PROFILE_CGRAPH
		Debug(((stringstream&)(stringstream() << "total for " << counter_cgraph << " calls to ConstraintGraphSolver::Solve = " << timer_total << endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "collect_and_count =\t\t"	<< timer_collect_and_count	<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "vdata_init =\t\t\t"		<< timer_vdata_init			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "make_batches =\t\t\t"		<< timer_make_batches		<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "batch_vbos =\t\t\t"		<< timer_batch_vbos			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "process =\t\t\t\t"		<< timer_process			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "retrieve =\t\t\t\t"		<< timer_retrieve			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_collect_and_count + timer_vdata_init + timer_make_batches + timer_process + timer_retrieve << endl)).str());
#endif
	}

	void ConstraintGraphSolver::Init(ContentMan* content)
	{
		Cache<Shader>* shader_cache = content->GetCache<Shader>();

		// initialize stuff pertaining to constraint evaluator HAC
		mass_infos = new VertexBuffer();
		mass_infos->AddAttribute("data", Float, 4);
		mass_info_tex = new TextureBuffer(mass_infos, GL_RGBA32F);					// bunch of data packed into a handful of consecutive texels!

		constraint_data = new VertexBuffer();
		constraint_data->AddAttribute("data", Float, 4);
		constraints_tex = new TextureBuffer(constraint_data, GL_RGBA32F);			// bunch of data packed into a handful of consecutive texels!

		velocity_data_a = new VertexBuffer();
		velocity_data_a->AddAttribute("rot", Float, 4);
		velocity_data_a->AddAttribute("vel", Float, 4);

		map<string, string> vdata_copy_output_map;
		vdata_copy_output_map["out_rot"] = "rot";
		vdata_copy_output_map["out_vel"] = "vel";

		velocity_data_b = VertexBuffer::CreateEmptyCopyAttributes(velocity_data_a);

		constraint_eval_hac = new HardwareAcceleratedComputation(shader_cache->Load("constraint_eval-v"), vdata_copy_output_map, velocity_data_a);
		ShaderProgram* constraint_eval_prog = constraint_eval_hac->shader_program;

		if(!constraint_eval_prog)
			return;

		constraint_eval_prog->AddUniform<float>(			new UniformFloat(			"timestep"							));
		constraint_eval_prog->AddUniform<int>(				new UniformInt(				"num_rigid_bodies"					));
		constraint_eval_prog->AddUniform<TextureBuffer>(	new UniformTextureBuffer(	"constraint_data",				0	));
		constraint_eval_prog->AddUniform<TextureBuffer>(	new UniformTextureBuffer(	"velocity_data",				1	));
		constraint_eval_prog->AddUniform<TextureBuffer>(	new UniformTextureBuffer(	"mass_infos",					2	));

		// set these once and be done with them (the corresponding VertexBuffer pointers won't change)
		constraint_eval_prog->SetUniform<TextureBuffer>( "mass_infos",      mass_info_tex   );
		constraint_eval_prog->SetUniform<TextureBuffer>( "constraint_data", constraints_tex );

		vdata_tex_a = new TextureBuffer(velocity_data_a, GL_RGBA32F);
		vdata_tex_b = new TextureBuffer(velocity_data_b, GL_RGBA32F);

		init_ok = true;
	}

	void ConstraintGraphSolver::Solve(float timestep, unsigned int iterations, vector<PhysicsConstraint*>& constraints)
	{
#if PROFILE_CGRAPH
		ProfilingTimer timer, timer2;
		timer2.Start();
		timer.Start();

		++counter_cgraph;
#endif

		if(!init_ok)
			return;

		// collect unique rigid bodies in a vector, and map to each one its index... and while we're at it, tally how many constraints there are of each type
		static vector<RigidBody*> rigid_bodies;
		boost::unordered_map<RigidBody*, unsigned int> rb_indices;
		unsigned int joint_constraints = 0, contact_points = 0;

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

			if(dynamic_cast<JointConstraint*>(constraint))
				++joint_constraints;
			else
				++contact_points;
		}
		unsigned int num_rigid_bodies = rigid_bodies.size();

#if PROFILE_CGRAPH
		timer_collect_and_count += timer.GetAndRestart();
#endif


		// put rigid bodies' velocity data and mass infos into their respective vertex buffers
		velocity_data_a->SetNumVerts(num_rigid_bodies);
		velocity_data_b->SetNumVerts(num_rigid_bodies);
		mass_infos->SetNumVerts(num_rigid_bodies * 4);			// 13 floats rounds up to 4 texels

		float* lv_ptr = velocity_data_a->GetFloatPointer("vel");
		float* av_ptr = velocity_data_a->GetFloatPointer("rot");
		float* mi_ptr = mass_infos->GetFloatPointer("data");
		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(), rb_end = rigid_bodies.end(); iter != rb_end; ++iter)
		{
			RigidBody* body = *iter;

			// copy velocity data
			Vec3 vel = body->GetLinearVelocity(), rot = body->GetAngularVelocity();

			memcpy(lv_ptr, &vel, 3 * sizeof(float));
			memcpy(av_ptr, &rot, 3 * sizeof(float));

			lv_ptr += 4;								// 3 floats, + 1 wasted float in each texel
			av_ptr += 4;

			// copy mass info ... or rather, "inverse mass info" :3
			*(mi_ptr++) = body->inv_mass;

			memcpy(mi_ptr, &body->cached_com,		3 * sizeof(float));
			mi_ptr += 3;

			memcpy(mi_ptr, body->inv_moi.values,	9 * sizeof(float));
			mi_ptr += 9 + 3;							// 9 for the matrix we just copied, + 3 wasted floats in the last texel
		}
		velocity_data_a->BuildVBO();
		velocity_data_b->BuildVBO();
		mass_infos->BuildVBO();

#if PROFILE_CGRAPH
		timer_vdata_init += timer.GetAndRestart();
#endif

		constraint_data->SetNumVerts(joint_constraints * TEXELS_PER_JC + contact_points * TEXELS_PER_CP);
		float* constraint_data_ptr = constraint_data->GetFloatPointer("data");
		unsigned int constraint_texel_index = 0;

		vector<BatchData> batches;
		vector<PhysicsConstraint*> unassigned = constraints;
		while(!unassigned.empty())
			batches.push_back(BatchData(unassigned, rb_indices, constraint_data_ptr, constraint_texel_index));

		Debug(((stringstream&)(stringstream() << "batches = " << batches.size() << endl)).str());

#if PROFILE_CGRAPH
		timer_make_batches += timer.GetAndRestart();
#endif

		constraint_data->BuildVBO();

		for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			iter->v_xfer_indices->BuildVBO();

#if PROFILE_CGRAPH
		timer_batch_vbos += timer.GetAndRestart();
#endif


		VertexBuffer *active_vdata = velocity_data_a, *inactive_vdata = velocity_data_b;
		TextureBuffer *active_vtex = vdata_tex_a, *inactive_vtex = vdata_tex_b;

		// do the actual solving
		ShaderProgram* constraint_eval_prog = constraint_eval_hac->shader_program;
		
		int velocity_data_size = (int)num_rigid_bodies;
		constraint_eval_prog->SetUniform<float>(	"timestep",			&timestep			);
		constraint_eval_prog->SetUniform<int>(		"num_rigid_bodies",	&velocity_data_size	);

		for(unsigned int i = 0; i < iterations; ++i)
			for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			{
				BatchData& batch = *iter;

				// do constraint shader stuff
				constraint_eval_prog->SetUniform<TextureBuffer>("velocity_data", active_vtex);
				constraint_eval_hac->Process(batch.v_xfer_indices, inactive_vdata);

				// change which direction the copying is going (back and forth)... can't use one buffer as both input and output or it will be undefined behavior!
				swap(active_vdata, inactive_vdata);
				swap(active_vtex, inactive_vtex);
			}

#if PROFILE_CGRAPH
		timer_process += timer.GetAndRestart();
#endif

		// copy linear and angular velocity data from vertex buffer back to the corresponding RigidBody objects
		active_vdata->UpdateDataFromGL();

		lv_ptr = active_vdata->GetFloatPointer("vel");
		av_ptr = active_vdata->GetFloatPointer("rot");

		for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
		{
			RigidBody* body = rigid_bodies[i];

			Vec3 vel, rot;
			memcpy(&vel, lv_ptr, 3 * sizeof(float));
			memcpy(&rot, av_ptr, 3 * sizeof(float));

			body->SetLinearVelocity(vel);
			body->SetAngularVelocity(rot);

			lv_ptr += 4;
			av_ptr += 4;
		}

		// clean up per-batch stuff
		for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			iter->Cleanup();

		rigid_bodies.clear();

#if PROFILE_CGRAPH
		timer_retrieve += timer.Stop();
		timer_total += timer2.Stop();
#endif
	}
}

