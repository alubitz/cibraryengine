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

			VertexBuffer* output_proto;
			vector<string>				varying_names;		// needed to keep the vector<const GLchar*> from getting corrupted
			vector<const GLchar*>		var_name;
			vector<unsigned int>		var_n_per_vert;
			vector<VertexAttributeType>	var_type;

			map<string, string>			output_mapping;

			bool init_ok;

			bool InitShaderProgram();

		public:

			Shader* shader;
			ShaderProgram* shader_program;

			HardwareAcceleratedComputation(Shader* shader, map<string, string>& output_mapping, VertexBuffer* output_proto);
			~HardwareAcceleratedComputation();

			void Process(VertexBuffer* input_data, VertexBuffer* output_data);
	};
}
