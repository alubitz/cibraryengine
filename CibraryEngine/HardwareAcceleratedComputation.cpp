#include "StdAfx.h"
#include "HardwareAcceleratedComputation.h"

#include "Vector.h"
#include "DebugLog.h"

#include "Shader.h"
#include "VertexBuffer.h"

#include "ProfilingTimer.h"

#define PROFILE_HAC_PROCESS 0

namespace CibraryEngine
{
#if PROFILE_HAC_PROCESS
	static float timer_resize = 0.0f;
	static float timer_setshader_bind = 0.0f;
	static float timer_draw = 0.0f;
	static float timer_query_verts = 0.0f;
	static float timer_resize_outs = 0.0f;
	static float timer_extract_data = 0.0f;

	static unsigned int counter_hac_process = 0;
#endif




	/*
	 * HardwareAcceleratedComputation methods
	 */
	HardwareAcceleratedComputation::HardwareAcceleratedComputation(Shader* shader, vector<const GLchar*>& varying_names) :
		shader(shader),
		shader_program(NULL),
		output_vertex_array(0),
		output_channels(),
		query(0),
		init_ok(false),
		varying_names(varying_names)
	{
		if(!shader)
			return;

		glGenQueries(1, &query);

		if(!InitShaderProgram())	{ DEBUG(); return; }
		if(!InitArrayBuffers())		{ DEBUG(); return; }
		if(!InitVertexArrays())		{ DEBUG(); return; }

		GLDEBUG();

		init_ok = true;
	}

	HardwareAcceleratedComputation::~HardwareAcceleratedComputation()
	{
		GLDEBUG();

		if(shader_program) { shader_program->Dispose(); delete shader_program; shader_program = NULL; }

		glDeleteVertexArrays(1, &output_vertex_array);

		glDeleteBuffers(output_channels.size(), output_channels.data());
		output_channels.clear();

		glDeleteQueries(1, &query);

#if PROFILE_HAC_PROCESS
		Debug(((stringstream&)(stringstream() << "time usage during " << counter_hac_process << " calls to HardwareAcceleratedComputation::Process" << endl)).str());
		Debug(((stringstream&)(stringstream() << "\tresize =\t\t\t\t"		<< timer_resize			<< endl)).str());
		Debug(((stringstream&)(stringstream() << "\tsetshader_bind =\t\t"	<< timer_setshader_bind	<< endl)).str());
		Debug(((stringstream&)(stringstream() << "\tdraw =\t\t\t\t\t"		<< timer_draw			<< endl)).str());
		Debug(((stringstream&)(stringstream() << "\tquery_verts =\t\t\t"	<< timer_query_verts	<< endl)).str());
		Debug(((stringstream&)(stringstream() << "\tresize_outs =\t\t\t"	<< timer_resize_outs	<< endl)).str());
		Debug(((stringstream&)(stringstream() << "\textract_data =\t\t\t"	<< timer_extract_data	<< endl)).str());
		Debug(((stringstream&)(stringstream() << "\ttotal of above =\t\t"	<< timer_resize + timer_setshader_bind + timer_draw + timer_query_verts + timer_resize_outs + timer_extract_data << endl)).str());
#endif

		GLDEBUG();
	}

	// create shader program
	bool HardwareAcceleratedComputation::InitShaderProgram()
	{
		shader->CompileShader();

		char vlog[1024];
		int vertex_status;
		glGetShaderInfoLog(shader->shader_id, 1024, NULL, vlog);
		glGetShaderiv(shader->shader_id, GL_COMPILE_STATUS, &vertex_status);
		if(!vertex_status)
		{
			DEBUG();
			Debug(vlog);
		}

		shader_program = new ShaderProgram(shader, NULL);
		shader_program->program_id = glCreateProgram();
		glAttachShader(shader_program->program_id, shader->shader_id);

		// tell the transform feedback thing which varyings we are interested in
		glTransformFeedbackVaryings(shader_program->program_id, varying_names.size(), varying_names.data(), GL_SEPARATE_ATTRIBS);

		// required for the above call to glTransformFeedbackVaryings to take effect
		glLinkProgram(shader_program->program_id);

		GLDEBUG();


		char name[64];				// name is unused
		GLsizei length(0);			// length is unused
		GLsizei size(0);
		GLenum type(0);

		// TODO: have some sorta data type spec, modify the checking appropriately
		for(unsigned int i = 0; i < varying_names.size(); ++i)
		{
			glGetTransformFeedbackVarying(shader_program->program_id, i, 64, &length, &size, &type, name);
			if(size != 1 || type != GL_FLOAT_VEC4)
			{
				DEBUG();
				return false;
			}

			GLDEBUG();
		}

		GLDEBUG();

		return true;
	}


	bool HardwareAcceleratedComputation::InitArrayBuffers()
	{
		GLDEBUG();

		if(output_channels.empty())
		{
			// generate output buffer objects
			for(unsigned int i = 0; i < varying_names.size(); ++i)
			{
				unsigned int output_channel;
				glGenBuffers(1, &output_channel);
				output_channels.push_back(output_channel);

				GLDEBUG();
			}
		}

		return true;
	}

	void HardwareAcceleratedComputation::ResizeArrayBuffers(unsigned int num_verts)
	{
		for(unsigned int i = 0; i < output_channels.size(); ++i)
		{
			glBindBuffer(GL_ARRAY_BUFFER, output_channels[i]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * num_verts, NULL, GL_DYNAMIC_READ);
		}
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	bool HardwareAcceleratedComputation::InitVertexArrays()
	{
		GLDEBUG();

		// build the output vertex array objects
		glGenVertexArrays(1, &output_vertex_array);
		glBindVertexArray(output_vertex_array);

			for(unsigned int i = 0; i < output_channels.size(); ++i)
			{
				glBindBuffer(GL_ARRAY_BUFFER, output_channels[i]);
					glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, 0, NULL);		// TODO: generalize count and type params
			}
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			for(unsigned int i = 0; i < output_channels.size(); ++i)
				glEnableVertexAttribArray(i);

		glBindVertexArray(0);

		GLDEBUG();

		return true;
	}

	void HardwareAcceleratedComputation::Process(VertexBuffer* input_data, VertexBuffer* output_data)
	{
		if(!init_ok)
			return;

#if PROFILE_HAC_PROCESS
		++counter_hac_process;

		ProfilingTimer timer;
		timer.Start();
#endif

		GLDEBUG();

		ResizeArrayBuffers(input_data->GetNumVerts());

#if PROFILE_HAC_PROCESS
		timer_resize += timer.GetAndRestart();
#endif

		// preparation for transform feedback
		// disable rasterization; only process the vertices!
		glEnable(GL_RASTERIZER_DISCARD); GLDEBUG();

		ShaderProgram::SetActiveProgram(shader_program); GLDEBUG();

		for(unsigned int i = 0; i < output_channels.size(); ++i)
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, i, output_channels[i]);

#if PROFILE_HAC_PROCESS
		timer_setshader_bind += timer.GetAndRestart();
#endif

		// the actual transform feedback
		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query);
		glBeginTransformFeedback(GL_POINTS);

			GLDEBUG();
			input_data->Draw();
			GLDEBUG();

		glEndTransformFeedback();
		glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN); GLDEBUG();

		glDisable(GL_RASTERIZER_DISCARD); GLDEBUG();

#if PROFILE_HAC_PROCESS
		timer_draw += timer.GetAndRestart();
#endif

		// getting the results of the transform feedback we just did
		GLuint primitives_written = 0;
		glGetQueryObjectuiv(query, GL_QUERY_RESULT, &primitives_written);

		unsigned int verts_per_primitive = 1;
		unsigned int num_verts = primitives_written * verts_per_primitive;

#if PROFILE_HAC_PROCESS
		timer_resize_outs += timer.GetAndRestart();
#endif

		output_data->SetNumVerts(num_verts);

#if PROFILE_HAC_PROCESS
		timer_query_verts += timer.GetAndRestart();
#endif

		for(unsigned int i = 0; i < output_channels.size(); ++i)
		{
			unsigned int num_floats = num_verts * 4;			// TODO: get this number from somewhere; it could be different from one output variable to another

			float* data = output_data->GetFloatPointer(varying_names[i]);

			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, output_channels[i]);
			glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, num_floats * sizeof(float), data);
		}

		glUseProgram(0); GLDEBUG();

#if PROFILE_HAC_PROCESS
		timer_extract_data += timer.Stop();
#endif
	}
}
