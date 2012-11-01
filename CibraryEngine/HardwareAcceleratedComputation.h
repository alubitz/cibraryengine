#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class Shader;

	// TODO: generalize, then specialize!
	struct HardwareAcceleratedComputation
	{
		private:

			Shader* shader;

			GLuint shader_program;
			GLuint input_array_buffer;
			GLuint input_vertex_array;

			GLuint output_vertex_array;
			vector<GLuint> output_channels;			// array buffers which will go inside the output vertex array

			GLuint query;

			bool init_ok;

			bool InitShaderProgram();
			bool InitArrayBuffers();
			bool InitVertexArrays();

		public:

			vector<const GLchar*> varying_names;	// names of the output variables... indices are parallel to output_channels

			HardwareAcceleratedComputation(Shader* shader, vector<const GLchar*>& varying_names);
			~HardwareAcceleratedComputation();

			void Process();							// TODO: add a way to specify the input, and access the outputs
	};
}
