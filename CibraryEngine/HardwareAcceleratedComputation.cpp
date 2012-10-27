#include "StdAfx.h"
#include "HardwareAcceleratedComputation.h"

#include "Vector.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	// based on stuff i picked up from "gl-330-transform-feedback-separated.cpp" from g-truc.net
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
		transform_program(0),
		transform_array_buffer(0),
		transform_vertex_array(0),
		feedback_array_buffer_position(0),
		feedback_array_buffer_derp(0),
		feedback_vertex_array(0),
		query_object(0)
	{
	}

	// create shader program
	bool HardwareAcceleratedComputation::initProgram()
	{
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

		transform_program = glCreateProgram();
		glAttachShader(transform_program, vertex_shader);
		glDeleteShader(vertex_shader);

		// tell the transform feedback thing which varyings we are interested in
		vector<const GLchar*> varying_names;
		varying_names.push_back("gl_Position");
		varying_names.push_back("derp");
		glTransformFeedbackVaryings(transform_program, varying_names.size(), varying_names.data(), GL_SEPARATE_ATTRIBS);

		// required for the above call to glTransformFeedbackVaryings to take effect
		glLinkProgram(transform_program);


		int program_status;

		glGetProgramiv(transform_program, GL_LINK_STATUS, &program_status);

		if(!program_status)
		{
			char plog[1024];
			glGetProgramInfoLog(transform_program, 1024, NULL, plog);
			Debug(plog);
			return false;
		}

		GLDEBUG();

		char name[64];				// name is unused
		GLsizei length(0);			// length is unused
		GLsizei size(0);
		GLenum type(0);

		glGetTransformFeedbackVarying(transform_program, 0, 64, &length, &size, &type, name);
		if(size != 1 || type != GL_FLOAT_VEC4)
		{
			DEBUG();
			return false;
		}

		GLDEBUG();

		glGetTransformFeedbackVarying(transform_program, 1, 64, &length, &size, &type, name);
		if(size != 1 || type != GL_FLOAT_VEC4)
		{
			DEBUG();
			return false;
		}

		GLDEBUG();

		return true;
	}


	bool HardwareAcceleratedComputation::initArrayBuffer()
	{
		// generate some buffer objects
		glGenBuffers(1, &transform_array_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, transform_array_buffer);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * input_vert_count, input_vert_data, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glGenBuffers(1, &feedback_array_buffer_position);
		glBindBuffer(GL_ARRAY_BUFFER, feedback_array_buffer_position);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * input_vert_count, NULL, GL_DYNAMIC_READ);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glGenBuffers(1, &feedback_array_buffer_derp);
		glBindBuffer(GL_ARRAY_BUFFER, feedback_array_buffer_derp);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * input_vert_count, NULL, GL_DYNAMIC_READ);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		GLDEBUG();

		return true;
	}

	bool HardwareAcceleratedComputation::initVertexArray()
	{
		GLDEBUG();

		const unsigned int VSHADER_INPUT_ATTRIB_A = 0;

		const unsigned int VSHADER_OUTPUT_ATTRIB_A = 0;
		const unsigned int VSHADER_OUTPUT_ATTRIB_B = 1;

		// build the first vertex array object
		glGenVertexArrays(1, &transform_vertex_array);
		glBindVertexArray(transform_vertex_array);
			glBindBuffer(GL_ARRAY_BUFFER, transform_array_buffer);
				glVertexAttribPointer(VSHADER_INPUT_ATTRIB_A, 4, GL_FLOAT, GL_FALSE, 0, 0);
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			glEnableVertexAttribArray(VSHADER_INPUT_ATTRIB_A);
		glBindVertexArray(0);

		GLDEBUG();

		// build the second vertex array object
		glGenVertexArrays(1, &feedback_vertex_array);
		glBindVertexArray(feedback_vertex_array);
			glBindBuffer(GL_ARRAY_BUFFER, feedback_array_buffer_position);
				glVertexAttribPointer(VSHADER_OUTPUT_ATTRIB_A, 4, GL_FLOAT, GL_FALSE, 0, 0);
			glBindBuffer(GL_ARRAY_BUFFER, feedback_array_buffer_derp);
				glVertexAttribPointer(VSHADER_OUTPUT_ATTRIB_B, 4, GL_FLOAT, GL_FALSE, 0, 0);
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			glEnableVertexAttribArray(VSHADER_OUTPUT_ATTRIB_A);
			glEnableVertexAttribArray(VSHADER_OUTPUT_ATTRIB_B);
		glBindVertexArray(0);

		GLDEBUG();

		return true;
	}

	void HardwareAcceleratedComputation::begin()
	{
		glGenQueries(1, &query_object);

		if(!initProgram())		{ DEBUG(); return; }
		if(!initArrayBuffer())	{ DEBUG(); return; }
		if(!initVertexArray())	{ DEBUG(); return; }

		GLDEBUG();
	}

	void HardwareAcceleratedComputation::end()
	{
		GLDEBUG();

		glDeleteVertexArrays(1, &transform_vertex_array);
		glDeleteBuffers(1, &transform_array_buffer);
		glDeleteProgram(transform_program);

		glDeleteVertexArrays(1, &feedback_vertex_array);
		glDeleteBuffers(1, &feedback_array_buffer_position);
		glDeleteBuffers(1, &feedback_array_buffer_derp);

		glDeleteQueries(1, &query_object);

		GLDEBUG();
	}

	void HardwareAcceleratedComputation::display()
	{
		GLDEBUG();

		// preparation for transform feedback
		// disable rasterization; only process the vertices!
		glEnable(GL_RASTERIZER_DISCARD);

		GLDEBUG();

		glUseProgram(transform_program);
		GLDEBUG();

		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedback_array_buffer_position);
		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, feedback_array_buffer_derp);

		glBindVertexArray(transform_vertex_array);

		GLDEBUG();

		// the actual transform feedback
		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query_object);
		glBeginTransformFeedback(GL_POINTS);

			GLDEBUG();
			glDrawArrays(GL_POINTS, 0, input_vert_count);
			GLDEBUG();

		glEndTransformFeedback();
		glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);

		GLDEBUG();

		glDisable(GL_RASTERIZER_DISCARD);

		GLDEBUG();

		// getting the results of the transform feedback we just did
		GLuint primitives_written = 0;
		glGetQueryObjectuiv(query_object, GL_QUERY_RESULT, &primitives_written);

		unsigned int verts_per_primitive = 1;
		unsigned int num_verts = primitives_written * verts_per_primitive;
		unsigned int num_floats = num_verts * 4;

		float* position_data = new float[num_floats];
		float* derp_data = new float[num_floats];

		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedback_array_buffer_position);
		glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, num_floats * sizeof(float), position_data);

		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, feedback_array_buffer_derp);
		glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, num_floats * sizeof(float), derp_data);

		Debug(((stringstream&)(stringstream() << "Got data for " << num_verts << " verts" << endl)).str());
		for(unsigned int i = 0; i < num_floats; ++i)
		{
			switch(i % 4)
			{
				case 0:
					Debug(((stringstream&)(stringstream() << '\t' << "position[" << i / 4 << "] = (" << position_data[i] << ", ")).str());
					break;

				case 1:
				case 2:
					Debug(((stringstream&)(stringstream() << position_data[i] << ", ")).str());
					break;

				case 3:
					Debug(((stringstream&)(stringstream() << position_data[i] << ")" << endl)).str());
					break;
			}
		}
		for(unsigned int i = 0; i < num_floats; ++i)
		{
			switch(i % 4)
			{
				case 0:
					Debug(((stringstream&)(stringstream() << '\t' << "derp[" << i / 4 << "] = (" << derp_data[i] << ", ")).str());
					break;

				case 1:
				case 2:
					Debug(((stringstream&)(stringstream() << derp_data[i] << ", ")).str());
					break;

				case 3:
					Debug(((stringstream&)(stringstream() << derp_data[i] << ")" << endl)).str());
					break;
			}
		}

		delete[] position_data;
		delete[] derp_data;

		glUseProgram(0);

		GLDEBUG();
	}
}
