#include "StdAfx.h"
#include "HardwareAcceleratedComputation.h"

#include "Vector.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	// originally based on stuff from "gl-330-transform-feedback-separated.cpp" from g-truc.net; now heavily modified
	const GLsizei input_vert_count(5);
	const Vec4 input_vert_data[input_vert_count] =
	{
		Vec4(1,	2, 3, 4),
		Vec4(5,	6, 7, 8),
		Vec4(9,	0, 1, 2),
		Vec4(3,	4, 5, 6),
		Vec4(7, 8, 9, 0)
	};




	/*
	 * HardwareAcceleratedComputation methods
	 */
	HardwareAcceleratedComputation::HardwareAcceleratedComputation() :
		shader_program(0),
		input_array_buffer(0),
		input_vertex_array(0),
		output_vertex_array(0),
		output_channels(),
		query(0),
		varying_names()
	{
		// TODO: have this be specified via some external means
		varying_names.push_back("gl_Position");
		varying_names.push_back("derp");
	}

	// create shader program
	bool HardwareAcceleratedComputation::InitShaderProgram()
	{
		// TODO: have a way to input what shader to use (possibly as a Shader* parameter somewhere?)
		const GLchar* shader_source = "varying vec4 derp; void main() { gl_Position = gl_Vertex; derp = vec4(5, 5, 5, 5); }";

		// creating vertex shader from file
		GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertex_shader, 1, &shader_source, NULL);
		glCompileShader(vertex_shader);
		char vlog[1024];
		int vertex_status;
		glGetShaderInfoLog(vertex_shader, 1024, NULL, vlog);
		glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &vertex_status);
		if(!vertex_status)
		{
			DEBUG();
			Debug(vlog);
		}

		shader_program = glCreateProgram();
		glAttachShader(shader_program, vertex_shader);
		glDeleteShader(vertex_shader);

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
		// generate input buffer object
		glGenBuffers(1, &input_array_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, input_array_buffer);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * input_vert_count, input_vert_data, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		GLDEBUG();

		// generate output buffer objects
		for(unsigned int i = 0; i < varying_names.size(); ++i)
		{
			unsigned int output_channel;
			glGenBuffers(1, &output_channel);
			glBindBuffer(GL_ARRAY_BUFFER, output_channel);
				glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * input_vert_count, NULL, GL_DYNAMIC_READ);
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			output_channels.push_back(output_channel);

			GLDEBUG();
		}

		return true;
	}

	bool HardwareAcceleratedComputation::InitVertexArrays()
	{
		GLDEBUG();

		const unsigned int VSHADER_INPUT_ATTRIB_A = 0;

		const unsigned int VSHADER_OUTPUT_ATTRIB_A = 0;
		const unsigned int VSHADER_OUTPUT_ATTRIB_B = 1;

		// build the input vertex array object
		glGenVertexArrays(1, &input_vertex_array);
		glBindVertexArray(input_vertex_array);
			glBindBuffer(GL_ARRAY_BUFFER, input_array_buffer);
				glVertexAttribPointer(VSHADER_INPUT_ATTRIB_A, 4, GL_FLOAT, GL_FALSE, 0, 0);
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			glEnableVertexAttribArray(VSHADER_INPUT_ATTRIB_A);
		glBindVertexArray(0);

		GLDEBUG();

		// build the output vertex array objects
		glGenVertexArrays(1, &output_vertex_array);
		glBindVertexArray(output_vertex_array);

			for(unsigned int i = 0; i < output_channels.size(); ++i)
			{
				glBindBuffer(GL_ARRAY_BUFFER, output_channels[i]);
					glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, 0, 0);
			}
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			
			for(unsigned int i = 0; i < output_channels.size(); ++i)
				glEnableVertexAttribArray(i);

		glBindVertexArray(0);

		GLDEBUG();

		return true;
	}

	void HardwareAcceleratedComputation::Begin()
	{
		glGenQueries(1, &query);

		if(!InitShaderProgram())	{ DEBUG(); return; }
		if(!InitArrayBuffers())		{ DEBUG(); return; }
		if(!InitVertexArrays())		{ DEBUG(); return; }

		GLDEBUG();
	}

	void HardwareAcceleratedComputation::End()
	{
		GLDEBUG();

		glDeleteVertexArrays(1, &input_vertex_array);
		glDeleteBuffers(1, &input_array_buffer);
		glDeleteProgram(shader_program);

		glDeleteVertexArrays(1, &output_vertex_array);

		glDeleteBuffers(output_channels.size(), output_channels.data());
		output_channels.clear();

		glDeleteQueries(1, &query);

		GLDEBUG();
	}

	void HardwareAcceleratedComputation::Process()
	{
		GLDEBUG();

		// preparation for transform feedback
		// disable rasterization; only process the vertices!
		glEnable(GL_RASTERIZER_DISCARD); GLDEBUG();

		glUseProgram(shader_program); GLDEBUG();

		for(unsigned int i = 0; i < output_channels.size(); ++i)
			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, i, output_channels[i]);

		glBindVertexArray(input_vertex_array); GLDEBUG();

		// the actual transform feedback
		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query);
		glBeginTransformFeedback(GL_POINTS);

			GLDEBUG();
			glDrawArrays(GL_POINTS, 0, input_vert_count);
			GLDEBUG();

		glEndTransformFeedback();
		glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN); GLDEBUG();

		glDisable(GL_RASTERIZER_DISCARD); GLDEBUG();

		// getting the results of the transform feedback we just did
		GLuint primitives_written = 0;
		glGetQueryObjectuiv(query, GL_QUERY_RESULT, &primitives_written);

		unsigned int verts_per_primitive = 1;
		unsigned int num_verts = primitives_written * verts_per_primitive;

		Debug(((stringstream&)(stringstream() << "Got data for " << num_verts << " verts" << endl)).str());

		for(unsigned int i = 0; i < output_channels.size(); ++i)
		{
			unsigned int num_floats = num_verts * 4;			// could be different from one output variable to another
			float* data = new float[num_floats];

			glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, output_channels[i]);
			glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, num_floats * sizeof(float), data);

			string varying_name = varying_names[i];
		
			for(unsigned int j = 0; j < num_floats; ++j)
			{
				float datum = data[j];
				switch(j % 4)
				{
					case 0:
						Debug(((stringstream&)(stringstream() << '\t' << varying_name << "[" << j / 4 << "] = (" << datum << ", ")).str());
						break;

					case 1:
					case 2:
						Debug(((stringstream&)(stringstream() << datum << ", ")).str());
						break;

					case 3:
						Debug(((stringstream&)(stringstream() << datum << ")" << endl)).str());
						break;
				}
			}

			delete[] data;
		}

		glUseProgram(0); GLDEBUG();
	}
}
