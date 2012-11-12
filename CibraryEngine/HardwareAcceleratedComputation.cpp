#include "StdAfx.h"
#include "HardwareAcceleratedComputation.h"

#include "Vector.h"
#include "DebugLog.h"

#include "Shader.h"
#include "VertexBuffer.h"

#include "ProfilingTimer.h"

namespace CibraryEngine
{
	/*
	 * HardwareAcceleratedComputation methods
	 */
	HardwareAcceleratedComputation::HardwareAcceleratedComputation(Shader* shader, map<string, string>& output_mapping_, VertexBuffer* output_proto) :
		output_proto(VertexBuffer::CreateEmptyCopyAttributes(output_proto)),
		output_mapping(output_mapping_),
		query(0),
		init_ok(false),
		shader(shader),
		shader_program(NULL)
	{
		if(!shader)					{ DEBUG(); return; }

		// if nothing is mapped, map all attribute names in the output prototype to themselves
		if(output_mapping.empty())
		{
			vector<string> attributes = output_proto->GetAttributes();
			for(vector<string>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
				output_mapping[*iter] = *iter;
		}

		glGenQueries(1, &query);
		if(!InitShaderProgram())	{ DEBUG(); return; }

		GLDEBUG();

		init_ok = true;
	}

	HardwareAcceleratedComputation::~HardwareAcceleratedComputation()
	{
		GLDEBUG();

		if(output_proto)	{ output_proto->Dispose();		delete output_proto;	output_proto = NULL;	}
		if(shader_program)	{ shader_program->Dispose();	delete shader_program;	shader_program = NULL;	}

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

			return false;
		}

		shader_program = new ShaderProgram(shader, NULL);
		shader_program->program_id = glCreateProgram();
		glAttachShader(shader_program->program_id, shader->shader_id);

		vector<string> attribute_names = output_proto->GetAttributes();
		unsigned int num_attributes = attribute_names.size();
		for(map<string, string>::iterator iter = output_mapping.begin(); iter != output_mapping.end(); ++iter)
		{
			varying_names.push_back(iter->first);
			var_name.push_back(iter->first.c_str());
			var_n_per_vert.push_back(output_proto->GetAttribNPerVertex(iter->second));
			var_type.push_back(output_proto->GetAttribType(iter->second));
		}

		// tell the transform feedback thing which varyings we are interested in
		glTransformFeedbackVaryings(shader_program->program_id, var_name.size(), var_name.data(), GL_SEPARATE_ATTRIBS);

		// required for the above call to glTransformFeedbackVaryings to take effect
		glLinkProgram(shader_program->program_id);

		GLDEBUG();

		char name[64];
		GLsizei size = 0;
		GLenum type = 0;

		GLint num_varyings = 0;
		glGetProgramiv(shader_program->program_id, GL_TRANSFORM_FEEDBACK_VARYINGS, &num_varyings);

		for(int i = 0; i < num_varyings; ++i)
		{
			glGetTransformFeedbackVarying(shader_program->program_id, i, 64, NULL, &size, &type, name);

			// see which name this matches (not sure if necessary?)
			for(unsigned int j = 0; j < num_attributes; ++j)
			{
				if(strcmp(name, var_name[j]) == 0)
				{
					// found which varying this matches... now to check that it matches the prototype properly
					unsigned int wanted_n_per_vert = output_proto->GetAttribNPerVertex(name);
					VertexAttributeType wanted_type = output_proto->GetAttribType(name);

					unsigned int n_per_vert = 0;

					switch(var_type[j])
					{
						case Float:
						{
							switch(type)
							{
								case GL_FLOAT:		{ n_per_vert = size;		break; }
								case GL_FLOAT_VEC2:	{ n_per_vert = size * 2;	break; }
								case GL_FLOAT_VEC3:	{ n_per_vert = size * 3;	break; }
								case GL_FLOAT_VEC4:	{ n_per_vert = size * 4;	break; }

								default: { DEBUG(); return false; }
							}

							break;
						}

						case Int:
						{
							switch(type)
							{
								case GL_INT:		{ n_per_vert = size;		break; }
								case GL_INT_VEC2:	{ n_per_vert = size * 2;	break; }
								case GL_INT_VEC3:	{ n_per_vert = size * 3;	break; }
								case GL_INT_VEC4:	{ n_per_vert = size * 4;	break; }

								default: { DEBUG(); return false; }
							}
						}

						default: { DEBUG(); return false; }
					}

					if(var_n_per_vert[j] != n_per_vert) { DEBUG(); return false; }

					break;			// break out of search-for-name loop
				}
			}

			GLDEBUG();
		}

		GLDEBUG();

		return true;
	}

	void HardwareAcceleratedComputation::Process(VertexBuffer* input_data, VertexBuffer* output_data)
	{
		if(!init_ok)
			return;

		unsigned int num_input_verts = input_data->GetNumVerts();
		unsigned int output_vbo = output_data->GetVBO();

		GLDEBUG();

		ShaderProgram::SetActiveProgram(shader_program); GLDEBUG();

		// resize the output vbo
		glBindBuffer(GL_ARRAY_BUFFER, output_vbo);
		glBufferData(GL_ARRAY_BUFFER, num_input_verts * output_data->GetVertexSize(), NULL, GL_DYNAMIC_COPY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		GLDEBUG();

		vector<string> target_attribs = output_data->GetAttributes();
		unsigned int offset = 0;
		for(vector<string>::iterator iter = target_attribs.begin(); iter != target_attribs.end(); ++iter)
		{
			const string& attrib_name = *iter;
			VertexAttributeType attrib_type = output_data->GetAttribType(attrib_name);

			int attrib_size = 0;
			switch(attrib_type)
			{
				case Float:	{ attrib_size = output_data->GetAttribNPerVertex(attrib_name) * num_input_verts * sizeof(float);	break; }
				case Int:	{ attrib_size = output_data->GetAttribNPerVertex(attrib_name) * num_input_verts * sizeof(int);		break; }
			}

			if(attrib_size > 0)
			{
				unsigned int i = 0;
				for(map<string, string>::iterator jter = output_mapping.begin(); jter != output_mapping.end(); ++jter, ++i)
					if(jter->second == attrib_name)
					{
						GLDEBUG();

						glBindBufferRange(GL_TRANSFORM_FEEDBACK_BUFFER, i, output_vbo, offset, attrib_size);	GLDEBUG();
						break;
					}

				offset += attrib_size;
			}
		}

		GLDEBUG();

		// the actual transform feedback
		glEnable(GL_RASTERIZER_DISCARD); 

		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query);			GLDEBUG();
		glBeginTransformFeedback(GL_POINTS);									GLDEBUG();
		input_data->Draw();														GLDEBUG();
		glEndTransformFeedback();												GLDEBUG();
		glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);					GLDEBUG();

		glDisable(GL_RASTERIZER_DISCARD);										GLDEBUG();
		glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, 0);

		// getting the results of the transform feedback we just did...
		GLuint primitives_written = 0;
		glGetQueryObjectuiv(query, GL_QUERY_RESULT, &primitives_written);		// bizarrely, if this is skipped, it introduces undefined behavior :(

		// keep the size of the VBO wrapper up to date, ensuring UpdateDataFromGL will work properly
		output_data->SetNumVerts(num_input_verts);

		ShaderProgram::SetActiveProgram(NULL); GLDEBUG();
	}
}
