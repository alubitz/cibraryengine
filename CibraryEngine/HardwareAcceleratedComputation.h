#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	// TODO: generalize, then specialize!
	struct HardwareAcceleratedComputation
	{
		private:

			GLuint shader_program;
			GLuint input_array_buffer;
			GLuint input_vertex_array;

			GLuint output_vertex_array;
			vector<GLuint> output_channels;			// array buffers which will go inside the output vertex array

			GLuint query;

			bool InitShaderProgram();
			bool InitArrayBuffers();
			bool InitVertexArrays();

		public:

			vector<const GLchar*> varying_names;	// names of the output variables... indices parallel to output_channels

			HardwareAcceleratedComputation();

			void Begin();
			void End();

			void Process();
	};
}
