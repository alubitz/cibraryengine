#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class Shader;
	class ShaderProgram;
	struct VertexBuffer;

	enum VertexAttributeType;

	struct HardwareAcceleratedComputation
	{
		private:

			Shader* shader;
			ShaderProgram* shader_program;

			VertexBuffer* output_proto;
			vector<string>				attribute_names;		// needed to keep the vector<const GLchar*> from getting corrupted
			vector<const GLchar*>		attrib_name;
			vector<unsigned int>		attrib_n_per_vert;
			vector<VertexAttributeType>	attrib_type;

			GLuint query;

			bool init_ok;

			bool InitShaderProgram();

		public:

			HardwareAcceleratedComputation(Shader* shader, VertexBuffer* output_proto);
			~HardwareAcceleratedComputation();

			void Process(VertexBuffer* input_data, VertexBuffer* output_data);
	};
}
