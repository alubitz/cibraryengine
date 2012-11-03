#include "StdAfx.h"
#include "HardwareAcceleratedComputation.h"

#include "Vector.h"
#include "DebugLog.h"

#include "Shader.h"
#include "VertexBuffer.h"

namespace CibraryEngine
{
	/*
	 * HardwareAcceleratedComputation methods
	 */
	HardwareAcceleratedComputation::HardwareAcceleratedComputation(Shader* shader, vector<const GLchar*>& varying_names) :
		shader(shader),
		shader_program(0),
		output_vertex_array(0),
		output_channels(),
		query(0),
		init_ok(false),
		varying_names(varying_names)
	{
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

		glDeleteProgram(shader_program);

		glDeleteVertexArrays(1, &output_vertex_array);

		glDeleteBuffers(output_channels.size(), output_channels.data());
		output_channels.clear();

		glDeleteQueries(1, &query);

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

		shader_program = glCreateProgram();
		glAttachShader(shader_program, shader->shader_id);

		// tell the transform feedback thing which varyings we are interested in
		glTransformFeedbackVaryings(shader_program, varying_names.size(), varying_names.data(), GL_SEPARATE_ATTRIBS);

		// required for the above call to glTransformFeedbackVaryings to take effect
		glLinkProgram(shader_program);


		int program_status;
		glGetProgramiv(shader_program, GL_LINK_STATUS, &program_status);

		if(!program_status)
		{
			char plog[1024];
			glGetProgramInfoLog(shader_program, 1024, NULL, plog);
			Debug(plog);
			return false;
		}

		GLDEBUG();

		char name[64];				// name is unused
		GLsizei length(0);			// length is unused
		GLsizei size(0);
		GLenum type(0);

		// TODO: have some sorta data type spec, modify the checking appropriately
		for(unsigned int i = 0; i < varying_names.size(); ++i)
		{
			glGetTransformFeedbackVarying(shader_program, i, 64, &length, &size, &type, name);
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

		GLDEBUG();

		ResizeArrayBuffers(input_data->GetNumVerts());

		// preparation for transform feedback
		// disable rasterization; only process the vertices!
		glEnable(GL_RASTERIZER_DISCARD); GLDEBUG();

		glUseProgram(shader_program); GLDEBUG();

		for(unsigned int i = 0; i < output_channels.size(); ++i)
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, i, output_channels[i]);

		// the actual transform feedback
		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query);
		glBeginTransformFeedback(GL_POINTS);

			GLDEBUG();
			input_data->Draw();
			GLDEBUG();

		glEndTransformFeedback();
		glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN); GLDEBUG();

		glDisable(GL_RASTERIZER_DISCARD); GLDEBUG();

		// getting the results of the transform feedback we just did
		GLuint primitives_written = 0;
		glGetQueryObjectuiv(query, GL_QUERY_RESULT, &primitives_written);

		unsigned int verts_per_primitive = 1;
		unsigned int num_verts = primitives_written * verts_per_primitive;

		output_data->SetNumVerts(num_verts);

		for(unsigned int i = 0; i < output_channels.size(); ++i)
		{
			unsigned int num_floats = num_verts * 4;			// TODO: get this number from somewhere; it could be different from one output variable to another

			float* data = output_data->GetFloatPointer(varying_names[i]);

			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, output_channels[i]);
			glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, num_floats * sizeof(float), data);
		}

		glUseProgram(0); GLDEBUG();
	}
}
