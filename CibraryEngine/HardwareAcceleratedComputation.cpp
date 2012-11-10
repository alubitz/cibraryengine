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
	HardwareAcceleratedComputation::HardwareAcceleratedComputation(Shader* shader, VertexBuffer* output_proto) :
		shader(shader),
		shader_program(NULL),
		output_proto(VertexBuffer::CreateEmptyCopyAttributes(output_proto)),
		query(0),
		init_ok(false)
	{
		if(!shader)					{ DEBUG(); return; }

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

			return false;
		}

		shader_program = new ShaderProgram(shader, NULL);
		shader_program->program_id = glCreateProgram();
		glAttachShader(shader_program->program_id, shader->shader_id);

		attribute_names = output_proto->GetAttributes();
		unsigned int num_attributes = attribute_names.size();
		for(vector<string>::iterator iter = attribute_names.begin(); iter != attribute_names.end(); ++iter)
		{
			attrib_name.push_back(iter->c_str());
			attrib_n_per_vert.push_back(output_proto->GetAttribNPerVertex(*iter));
			attrib_type.push_back(output_proto->GetAttribType(*iter));
		}

		// tell the transform feedback thing which varyings we are interested in
		glTransformFeedbackVaryings(shader_program->program_id, attrib_name.size(), attrib_name.data(), GL_SEPARATE_ATTRIBS);

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
				if(strcmp(name, attrib_name[j]) == 0)
				{
					// found which varying this matches... now to check that it matches the prototype properly
					unsigned int wanted_n_per_vert = output_proto->GetAttribNPerVertex(name);
					VertexAttributeType wanted_type = output_proto->GetAttribType(name);

					unsigned int n_per_vert = 0;

					switch(attrib_type[j])
					{
						case Float:
						{
							switch(type)
							{
								case GL_FLOAT:		{ n_per_vert = size;		break; }
								case GL_FLOAT_VEC2:	{ n_per_vert = size * 2;	break; }
								case GL_FLOAT_VEC3:	{ n_per_vert = size * 3;	break; }
								case GL_FLOAT_VEC4:	{ n_per_vert = size * 4;	break; }
										
								default:

									DEBUG();
									return false;
							}

							break;
						}

						default:
						{
							DEBUG();
							return false;
						}
					}

					if(attrib_n_per_vert[j] != n_per_vert)
					{
						DEBUG();
						return false;
					}

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

#if PROFILE_HAC_PROCESS
		++counter_hac_process;

		ProfilingTimer timer;
		timer.Start();
#endif

		GLDEBUG();

#if PROFILE_HAC_PROCESS
		timer_resize += timer.GetAndRestart();
#endif

		// preparation for transform feedback! disable rasterization; only process the vertices!
		glEnable(GL_RASTERIZER_DISCARD); GLDEBUG();

		ShaderProgram::SetActiveProgram(shader_program); GLDEBUG();

		// resize the output vbo
		glBindBuffer(GL_ARRAY_BUFFER, output_vbo);
		glBufferData(GL_ARRAY_BUFFER, num_input_verts * output_data->GetVertexSize(), NULL, GL_DYNAMIC_COPY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		vector<string> target_attribs = output_data->GetAttributes();
		unsigned int offset = 0;
		for(vector<string>::iterator iter = target_attribs.begin(); iter != target_attribs.end(); ++iter)
		{
			GLDEBUG();

			const string& attrib_name = *iter;
			VertexAttributeType attrib_type = output_data->GetAttribType(attrib_name);

			int attrib_size = 0;
			if(attrib_type == Float)
				attrib_size = sizeof(float) * output_data->GetAttribNPerVertex(attrib_name) * num_input_verts;

			if(attrib_size > 0)
			{
				unsigned int i = 0;
				for(vector<string>::iterator jter = attribute_names.begin(); jter != attribute_names.end(); ++jter, ++i)
					if(attrib_name == *jter)
					{
						GLDEBUG();

						glBindBufferRange(GL_TRANSFORM_FEEDBACK_BUFFER, i, output_vbo, offset, attrib_size);

						GLDEBUG();
						break;
					}

				offset += attrib_size;
			}
		}

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

		// ensure UpdateDataFromGL will work properly
		output_data->SetNumVerts(num_input_verts);

		ShaderProgram::SetActiveProgram(NULL); GLDEBUG();

#if PROFILE_HAC_PROCESS
		timer_extract_data += timer.Stop();
#endif
	}
}
