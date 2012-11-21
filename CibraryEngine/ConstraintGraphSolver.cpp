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
		v_xfer_indices(0)
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

		unsigned int num_rigid_bodies = rb_indices.size();

		// init velocity transfer indices buffer (used as a uniform buffer texture)
		glGenBuffers(1, &v_xfer_indices);
		glBindBuffer(GL_ARRAY_BUFFER, v_xfer_indices);
		glBufferData(GL_ARRAY_BUFFER, num_rigid_bodies * 3 * sizeof(float), NULL, GL_DYNAMIC_DRAW);

		float* constraint_index_array = new float[num_rigid_bodies];
		memset(constraint_index_array, 0, num_rigid_bodies * sizeof(float));

		float* object_index_array = new float[num_rigid_bodies * 2];

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

		glBufferSubData(GL_ARRAY_BUFFER, 0,									num_rigid_bodies * sizeof(float),		constraint_index_array);
		glBufferSubData(GL_ARRAY_BUFFER, num_rigid_bodies * sizeof(float),	num_rigid_bodies * 2 * sizeof(float),	object_index_array);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		delete[] constraint_index_array;
		delete[] object_index_array;
	}

	void ConstraintGraphSolver::BatchData::Cleanup() { if(v_xfer_indices) { glDeleteBuffers(1, &v_xfer_indices); v_xfer_indices = 0; } }




	/*
	 * ConstraintGraphSolver methods
	 */
	ConstraintGraphSolver::ConstraintGraphSolver() :
		shader(0),
		program(0),
		u_timestep(-1),
		u_num_rigid_bodies(-1),
		u_constraint_data(-1),
		u_velocity_data(-1),
		u_mass_infos(-1),
		velocity_data_a(0),
		velocity_data_b(0),
		constraint_data(0),
		mass_infos(0),
		vdata_tex_a(0),
		vdata_tex_b(0),
		constraints_tex(0),
		mass_info_tex(0),
		init_ok(false)
	{
	}

	ConstraintGraphSolver::~ConstraintGraphSolver()
	{
		if(program)				{ glDeleteProgram(program);					program = 0;			}
		if(shader)				{ glDeleteShader(shader);					shader = 0;				}

		if(vdata_tex_a)			{ glDeleteTextures(1, &vdata_tex_a);		vdata_tex_a = 0;		}
		if(vdata_tex_b)			{ glDeleteTextures(1, &vdata_tex_b);		vdata_tex_b = 0;		}
		if(constraints_tex)		{ glDeleteTextures(1, &constraints_tex);	constraints_tex = 0;	}
		if(mass_info_tex)		{ glDeleteTextures(1, &mass_info_tex);		mass_info_tex = 0;		}

		if(velocity_data_a)		{ glDeleteBuffers(1, &velocity_data_a);		velocity_data_a = 0;	}
		if(velocity_data_b)		{ glDeleteBuffers(1, &velocity_data_b);		velocity_data_b = 0;	}
		if(constraint_data)		{ glDeleteBuffers(1, &constraint_data);		constraint_data = 0;	}
		if(mass_infos)			{ glDeleteBuffers(1, &mass_infos);			mass_infos = 0;			}

#if PROFILE_CGRAPH
		Debug(((stringstream&)(stringstream() << "total for " << counter_cgraph << " calls to ConstraintGraphSolver::Solve = " << timer_total << endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "collect_and_count =\t\t"	<< timer_collect_and_count	<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "vdata_init =\t\t\t"		<< timer_vdata_init			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "make_batches =\t\t\t"		<< timer_make_batches		<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "process =\t\t\t\t"		<< timer_process			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "retrieve =\t\t\t\t"		<< timer_retrieve			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_collect_and_count + timer_vdata_init + timer_make_batches + timer_process + timer_retrieve << endl)).str());
#endif
	}

	void ConstraintGraphSolver::Init(ContentMan* content)
	{
		Cache<Shader>* shader_cache = content->GetCache<Shader>();

		// initialize stuff pertaining to constraint evaluator HAC
		glGenBuffers(1, &mass_infos);
		glGenTextures(1, &mass_info_tex);

		glGenBuffers(1, &constraint_data);
		glGenTextures(1, &constraints_tex);

		glGenBuffers(1, &velocity_data_a);
		glGenBuffers(1, &velocity_data_b);

		glGenTextures(1, &vdata_tex_a);
		glGenTextures(1, &vdata_tex_b);

		string shader_source;
		if(GetFileString("Files/Shaders/constraint_eval-v.txt", &shader_source))			// nonzero = failure code
			return;
		const char* source_string = shader_source.c_str();

		shader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(shader, 1, &source_string, NULL);
		glCompileShader(shader);

		program = glCreateProgram();
		glAttachShader(program, shader);

		const GLchar* var_names[] = { "out_rot", "out_vel" };
		glTransformFeedbackVaryings(program, 2, var_names, GL_SEPARATE_ATTRIBS);

		glLinkProgram(program);

		char vlog[1024];
		char plog[1024];
		glGetShaderInfoLog(shader, 1024, NULL, vlog);
		glGetProgramInfoLog(program, 1024, NULL, plog);

		int vertex_status, program_status;

		glGetShaderiv(shader, GL_COMPILE_STATUS, &vertex_status);
		glGetProgramiv(program, GL_LINK_STATUS, &program_status);

		GLDEBUG();

		if(!vertex_status)
		{
			Debug(vlog);

			glDeleteShader(shader);
			program = 0;

			return;
		}
		if(!program_status)
		{
			Debug(plog);

			glDeleteProgram(program);
			shader = 0;

			return;
		}

		GLDEBUG();

		char name[64];
		GLsizei size = 0;
		GLenum type = 0;

		GLint num_varyings = 0;
		glGetProgramiv(program, GL_TRANSFORM_FEEDBACK_VARYINGS, &num_varyings);

		for(int i = 0; i < num_varyings; ++i)
		{
			glGetTransformFeedbackVarying(program, i, 64, NULL, &size, &type, name);

			// see which name this matches (not sure if necessary?)
			for(unsigned int j = 0; j < 2; ++j)
			{
				if(strcmp(name, var_names[j]) == 0)
				{
					// found which varying this matches... now to check that it matches the prototype properly
					unsigned int n_per_vert;

					switch(type)
					{
						case GL_FLOAT:		{ n_per_vert = size;		break; }
						case GL_FLOAT_VEC2:	{ n_per_vert = size * 2;	break; }
						case GL_FLOAT_VEC3:	{ n_per_vert = size * 3;	break; }
						case GL_FLOAT_VEC4:	{ n_per_vert = size * 4;	break; }

						default: { DEBUG(); return; }
					}


					if(n_per_vert != 4) { DEBUG(); return; }

					break;			// break out of search-for-name loop
				}
			}

			GLDEBUG();
		}

		u_timestep =				glGetUniformLocation(program, "timestep"			);
		u_num_rigid_bodies =		glGetUniformLocation(program, "num_rigid_bodies"	);
		u_constraint_data =			glGetUniformLocation(program, "constraint_data"		);
		u_velocity_data =			glGetUniformLocation(program, "velocity_data"		);
		u_mass_infos =				glGetUniformLocation(program, "mass_infos"			);

		a_constraint_data_index =	glGetAttribLocation(program, "constraint_data_index");
		a_object_indices =			glGetAttribLocation(program, "object_indices"		);

		glActiveTexture(GL_TEXTURE0);

		GLDEBUG();

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

		GLDEBUG();

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
		float* velocity_data_array = new float[num_rigid_bodies * 8];
		float* mass_infos_array = new float[num_rigid_bodies * 16];

		float* lv_ptr = velocity_data_array + num_rigid_bodies * 4;
		float* av_ptr = velocity_data_array;
		float* mi_ptr = mass_infos_array;
		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(), rb_end = rigid_bodies.end(); iter != rb_end; ++iter)
		{
			RigidBody* body = *iter;

			// copy velocity data
			memcpy(lv_ptr, &body->vel, 3 * sizeof(float));
			memcpy(av_ptr, &body->rot, 3 * sizeof(float));

			lv_ptr += 4;								// 3 floats, + 1 wasted float in each texel
			av_ptr += 4;

			// copy mass info ... or rather, "inverse mass info" :3
			*(mi_ptr++) = body->inv_mass;

			memcpy(mi_ptr, &body->cached_com,		3 * sizeof(float));
			mi_ptr += 3;

			memcpy(mi_ptr, body->inv_moi.values,	9 * sizeof(float));
			mi_ptr += 9 + 3;							// 9 for the matrix we just copied, + 3 wasted floats in the last texel
		}
		glBindBuffer(GL_ARRAY_BUFFER, velocity_data_a);
		glBufferData(GL_ARRAY_BUFFER, num_rigid_bodies * 8 * sizeof(float), velocity_data_array, GL_STREAM_COPY);
		glBindBuffer(GL_ARRAY_BUFFER, velocity_data_b);
		glBufferData(GL_ARRAY_BUFFER, num_rigid_bodies * 8 * sizeof(float), NULL, GL_STREAM_COPY);
		glBindBuffer(GL_ARRAY_BUFFER, mass_infos);
		glBufferData(GL_ARRAY_BUFFER, num_rigid_bodies * 16 * sizeof(float), mass_infos_array, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

#if PROFILE_CGRAPH
		timer_vdata_init += timer.GetAndRestart();
#endif

		unsigned int constraint_data_texels = joint_constraints * TEXELS_PER_JC + contact_points * TEXELS_PER_CP;
		float* constraint_data_array = new float[constraint_data_texels * 4];
		float* constraint_data_ptr = constraint_data_array;
		unsigned int constraint_texel_index = 0;

		vector<BatchData> batches;
		vector<PhysicsConstraint*> unassigned = constraints;
		while(!unassigned.empty())
			batches.push_back(BatchData(unassigned, rb_indices, constraint_data_ptr, constraint_texel_index));

		Debug(((stringstream&)(stringstream() << "batches = " << batches.size() << endl)).str());

		glBindBuffer(GL_ARRAY_BUFFER, constraint_data);
		glBufferData(GL_ARRAY_BUFFER, constraint_data_texels * 4 * sizeof(float), constraint_data_array, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

#if PROFILE_CGRAPH
		timer_make_batches += timer.GetAndRestart();
#endif

		GLDEBUG();

		unsigned int active_vdata = velocity_data_a, inactive_vdata = velocity_data_b;
		unsigned int active_vtex = vdata_tex_a, inactive_vtex = vdata_tex_b;

		// do the actual solving
		glEnable(GL_RASTERIZER_DISCARD);
		glUseProgram(program);

		glUniform1f(u_timestep, timestep);
		glUniform1i(u_num_rigid_bodies, num_rigid_bodies);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_BUFFER, constraints_tex);
		glTexBufferARB(GL_TEXTURE_BUFFER, GL_RGBA32F, constraint_data);
		glUniform1i(u_constraint_data, 0);

		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_BUFFER, mass_info_tex);
		glTexBufferARB(GL_TEXTURE_BUFFER, GL_RGBA32F, mass_infos);
		glUniform1i(u_mass_infos, 2);

		GLDEBUG();

		glEnableVertexAttribArray((GLuint)a_constraint_data_index);
		glEnableVertexAttribArray((GLuint)a_object_indices);

		GLDEBUG();

		for(unsigned int i = 0; i < iterations; ++i)
			for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			{
				BatchData& batch = *iter;

				// do constraint shader stuff
				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_BUFFER, active_vtex);
				glTexBufferEXT(GL_TEXTURE_BUFFER, GL_RGBA32F, active_vdata);
				glUniform1i(u_velocity_data, 1);

				GLDEBUG();

				// set up outputs for transform feedback
				glBindBufferRange(GL_TRANSFORM_FEEDBACK_BUFFER, 0, inactive_vdata, 0,										num_rigid_bodies * 4 * sizeof(float));
				glBindBufferRange(GL_TRANSFORM_FEEDBACK_BUFFER, 1, inactive_vdata, num_rigid_bodies * 4 * sizeof(float),	num_rigid_bodies * 4 * sizeof(float));

				GLDEBUG();

				glBeginTransformFeedback(GL_POINTS);
				
					glBindBuffer(GL_ARRAY_BUFFER, batch.v_xfer_indices);
					glVertexAttribPointer((GLuint)a_constraint_data_index,	1, GL_FLOAT, false, 0, (void*)0);
					glVertexAttribPointer((GLuint)a_object_indices,			2, GL_FLOAT, false, 0, (void*)(num_rigid_bodies * sizeof(float)));
					glDrawArrays(GL_POINTS, 0, num_rigid_bodies);
				
				glEndTransformFeedback();

				GLDEBUG();

				glFlush();

				// change which direction the copying is going (back and forth)... can't use one buffer as both input and output or it will be undefined behavior!
				swap(active_vdata, inactive_vdata);
				swap(active_vtex, inactive_vtex);
			}

		GLDEBUG();

		glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, 0);

		glDisableVertexAttribArray((GLuint)a_constraint_data_index);
		glDisableVertexAttribArray((GLuint)a_object_indices);

		glUseProgram(0);
		glDisable(GL_RASTERIZER_DISCARD);

		glActiveTexture(GL_TEXTURE0);

		GLDEBUG();

#if PROFILE_CGRAPH
		timer_process += timer.GetAndRestart();
#endif

		// copy linear and angular velocity data from vertex buffer back to the corresponding RigidBody objects
		glBindBuffer(GL_ARRAY_BUFFER, active_vdata);
		glGetBufferSubData(GL_ARRAY_BUFFER, 0, num_rigid_bodies * 8 * sizeof(float), velocity_data_array);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		lv_ptr = velocity_data_array + num_rigid_bodies * 4;
		av_ptr = velocity_data_array;

		for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
		{
			RigidBody* body = rigid_bodies[i];

			memcpy(&body->vel, lv_ptr, 3 * sizeof(float));
			memcpy(&body->rot, av_ptr, 3 * sizeof(float));

			lv_ptr += 4;
			av_ptr += 4;
		}

		// clean up per-batch stuff
		for(vector<BatchData>::iterator iter = batches.begin(); iter != batches.end(); ++iter)
			iter->Cleanup();

		rigid_bodies.clear();

		delete[] constraint_data_array;
		delete[] velocity_data_array;
		delete[] mass_infos_array;

		GLDEBUG();

#if PROFILE_CGRAPH
		timer_retrieve += timer.Stop();
		timer_total += timer2.Stop();
#endif
	}
}

