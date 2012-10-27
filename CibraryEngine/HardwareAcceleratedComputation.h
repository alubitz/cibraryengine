#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	// TODO: generalize, then specialize!
	struct HardwareAcceleratedComputation
	{	
		// prefix of "Transform" means it has to do with the first shader, the one that just does vertex processing
		GLuint transform_program;
		GLuint transform_array_buffer;
		GLuint transform_vertex_array;

		// prefix of "Feedback" means it has to do with the second shader, the one that takes the first shader's output and draws stuff based on it
		GLuint feedback_array_buffer_position;			// one channel of the output
		GLuint feedback_array_buffer_derp;				// another channel of the output
		GLuint feedback_vertex_array;					// object which will contain one or more channels of output

		GLuint query_object;

		HardwareAcceleratedComputation();

		bool initProgram();
		bool initArrayBuffer();
		bool initVertexArray();

		void begin();
		void end();

		void display();
	};
}
