#include "StdAfx.h"
#include "VertexBuffer.h"

#include "DebugLog.h"

#include "Shader.h"

namespace CibraryEngine
{
	/*
	 * VertexBuffer methods
	 */
	VertexBuffer::VertexBuffer(DrawMode storage_mode) :
		attributes(),
		attribute_data(),
		storage_mode(storage_mode),
		num_verts(0),
		allocated_size(0),
		vbo_id(0)
	{
	}

	void VertexBuffer::InnerDispose()
	{
		InvalidateVBO();

		vector<string> attribs = GetAttributes();
		for(vector<string>::iterator iter = attribs.begin(); iter != attribs.end(); ++iter)
			RemoveAttribute(*iter);

		SetNumVerts(0);
	}

	DrawMode VertexBuffer::GetStorageMode() { return storage_mode; }

	unsigned int VertexBuffer::GetNumVerts() { return num_verts; }

	void VertexBuffer::SetNumVerts(unsigned int verts)
	{
		if(verts > allocated_size)
			SetAllocatedSize(verts + 20);

		num_verts = verts;
	}

	void VertexBuffer::SetAllocatedSize(unsigned int verts)
	{
		if(verts > allocated_size)
		{
			allocated_size = verts;

			boost::unordered_map<string, VertexData> nu_attribute_data;
			for(boost::unordered_map<string, VertexData>::iterator iter = attribute_data.begin(); iter != attribute_data.end(); ++iter)
			{
				if(GetAttribType(iter->first) == Float)
				{
					int n_per_vertex = GetAttribNPerVertex(iter->first);
					float* new_data = new float[allocated_size * n_per_vertex];
					float* old_data = iter->second.floats;
					if(old_data != NULL)
					{
						for(unsigned int i = 0; i < num_verts * n_per_vertex; ++i)
							new_data[i] = old_data[i];
					}
					nu_attribute_data[iter->first] = VertexData(new_data);
				}
			}

			for(boost::unordered_map<string, VertexData>::iterator iter = attribute_data.begin(); iter != attribute_data.end(); ++iter)
			{
				if(GetAttribType(iter->first) == Float)
				{
					float* floats = iter->second.floats;
					delete[] floats;
				}
			}
			attribute_data.clear();

			attribute_data = nu_attribute_data;
		}
		else if(verts == 0)
		{
			allocated_size = 0;

			boost::unordered_map<string, VertexData> nu_attribute_data;
			for(boost::unordered_map<string, VertexData>::iterator iter = attribute_data.begin(); iter != attribute_data.end(); ++iter)
			{
				if(GetAttribType(iter->first) == Float)
				{
					float* old_data = iter->second.floats;
					if(old_data != NULL)
						delete[] old_data;
				}

				nu_attribute_data[iter->first] = VertexData();
			}

			attribute_data = nu_attribute_data;
		}

		if(verts < num_verts)
			num_verts = verts;
	}

	void VertexBuffer::AddAttribute(const string& name, VertexAttributeType type, int n_per_vertex)
	{
		RemoveAttribute(name);				// in case it already exists!

		VertexAttribute attribute = VertexAttribute(name, type, n_per_vertex);
		attributes[name] = attribute;

		int num_elements = n_per_vertex * allocated_size;

		if(type == Float)
			attribute_data[name].floats = num_elements > 0 ? new float[num_elements] : NULL;
	}

	void VertexBuffer::RemoveAttribute(const string& name)
	{
		boost::unordered_map<string, VertexAttribute>::iterator found_name = attributes.find(name);
		if(found_name != attributes.end())
		{
			VertexAttribute attribute = found_name->second;
			attributes.erase(found_name);

			boost::unordered_map<string, VertexData>::iterator found_attrib = attribute_data.find(name);
			if(found_attrib != attribute_data.end())
			{
				VertexData data = found_attrib->second;

				if(attribute.type == Float)
					delete[] data.floats;

				attribute_data.erase(found_attrib);
			}
		}
	}

	VertexAttributeType VertexBuffer::GetAttribType(const string& name)
	{
		boost::unordered_map<string, VertexAttribute>::iterator found_name = attributes.find(name);
		if(found_name != attributes.end())
			return found_name->second.type;
		else
			return BadVertexAttribute;
	}

	int VertexBuffer::GetAttribNPerVertex(const string& name)
	{
		boost::unordered_map<string, VertexAttribute>::iterator found_name = attributes.find(name);
		if(found_name != attributes.end())
			return found_name->second.n_per_vertex;
		else
			return -1;
	}

	vector<string> VertexBuffer::GetAttributes()
	{
		vector<string> results;
		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
			results.push_back(iter->first);
		return results;
	}

	int VertexBuffer::GetVertexSize()
	{
		int total_size = 0;
		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			VertexAttribute& attrib = iter->second;
			if(attrib.type == Float)
				total_size += sizeof(float) * attrib.n_per_vertex;
		}
		return total_size;
	}

	float* VertexBuffer::GetFloatPointer(const string& name)
	{
		if(GetAttribType(name) == Float)
			return attribute_data[name].floats;
		else
			return NULL;
	}

	void VertexBuffer::InvalidateVBO()
	{ 
		if(vbo_id != 0)
		{
			glDeleteBuffers(1, &vbo_id);
			vbo_id = 0;
		}
	}

	void VertexBuffer::BuildVBO()
	{
		GLDEBUG();

		int total_size = GetVertexSize();

		InvalidateVBO();					// just in case...

		// generate a vbo
		glGenBuffers(1, &vbo_id);
		glBindBuffer(GL_ARRAY_BUFFER, vbo_id);

		// set the total size (based on the value we computed earlier)
		glBufferData(GL_ARRAY_BUFFER, total_size * num_verts, NULL, GL_STATIC_DRAW);

		int offset = 0;
		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;

			int attrib_size = 0;
			if(attrib.type == Float)
				attrib_size = sizeof(float) * attrib.n_per_vertex;

			if(attrib_size > 0)
			{
				if(attrib.type == Float)
					glBufferSubData(GL_ARRAY_BUFFER, offset * num_verts, attrib_size * num_verts, attribute_data[attrib.name].floats);

				offset += attrib_size;
			}
		}

		glBindBuffer(GL_ARRAY_BUFFER, 0);						// don't leave hardware vbo on

		GLDEBUG();
	}
	
	unsigned int VertexBuffer::GetVBO()
	{
		if(!vbo_id) 
			BuildVBO(); 

		return vbo_id;
	}

	void VertexBuffer::UpdateDataFromGL()
	{
		if(!vbo_id)
			return;

		glBindBuffer(GL_ARRAY_BUFFER, vbo_id);

		int offset = 0;
		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;

			int attrib_size = 0;
			if(attrib.type == Float)
				attrib_size = sizeof(float) * attrib.n_per_vertex;

			if(attrib_size > 0)
			{
				if(attrib.type == Float)
					glGetBufferSubData(GL_ARRAY_BUFFER, offset * num_verts, attrib_size * num_verts, attribute_data[attrib.name].floats);

				offset += attrib_size;
			}
		}

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	// utility stuff... do this once
	string* multi_tex_names = NULL;
	void BuildMultiTexNames()
	{
		if(multi_tex_names == NULL)
		{
			int max_texture_units;
			glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &max_texture_units);

			multi_tex_names = new string[max_texture_units];

			for(int i = 0; i < max_texture_units; ++i)
				multi_tex_names[i] = ((stringstream&)(stringstream() << "gl_MultiTexCoord" << i)).str();
		}
	}

	void VertexBuffer::PreDrawEnable()
	{
		unsigned int vbo = GetVBO();

		glBindBuffer(GL_ARRAY_BUFFER, vbo);

		int offset = 0;
		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;
			const string& name = iter->first;
			const char* name_cstr = name.c_str();

			if(name.length() >= 3 && memcmp(name_cstr, "gl_", 3) == 0)
			{
				if(!strcmp(name_cstr, "gl_Vertex"))
				{
					glEnable(GL_VERTEX_ARRAY);
					glVertexPointer(attrib.n_per_vertex, (GLenum)attrib.type, 0,	(void*)(num_verts * offset));
				}
				else if(!strcmp(name_cstr, "gl_Normal"))
				{
					glEnable(GL_NORMAL_ARRAY);
					glNormalPointer((GLenum)attrib.type, 0, (void*)(num_verts * offset));
				}
				else if(!strcmp(name_cstr, "gl_Color"))
				{
					glEnable(GL_COLOR_ARRAY);
					glColorPointer(attrib.n_per_vertex, (GLenum)attrib.type, 0, (void*)(num_verts * offset));
				}
				else
				{
					int max_texture_units;
					glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &max_texture_units);

					for(int j = 0; j < max_texture_units; ++j)
					{
						if(name == multi_tex_names[j])
						{
							glClientActiveTexture(GL_TEXTURE0 + j);
							glEnable(GL_TEXTURE_COORD_ARRAY);
							glTexCoordPointer(attrib.n_per_vertex,	(GLenum)attrib.type, 0, (void*)(num_verts * offset));
						}
					}
				}
			}
			else if(ShaderProgram* shader = ShaderProgram::GetActiveProgram())
			{
				GLint index = glGetAttribLocation(shader->program_id, name_cstr);
				if(index != -1)
				{
					glEnableVertexAttribArray((GLuint)index);
					glVertexAttribPointer((GLuint)index, attrib.n_per_vertex, (GLenum)attrib.type, true, 0, (void*)(num_verts * offset));

					GLDEBUG();
				}
				else
					Debug(((stringstream&)(stringstream() << "Couldn't enable attribute \"" << name << "\"" << endl)).str());
			}

			if(attrib.type == Float)
				offset += attrib.n_per_vertex * sizeof(float);
		}

		GLDEBUG();
	}

	void VertexBuffer::PostDrawDisable()
	{
		GLDEBUG();

		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;
			const string& name = iter->first;
			const char* name_cstr = name.c_str();

			if(name.length() >= 3 && memcmp(name_cstr, "gl_", 3) == 0)
			{
				if(!strcmp(name_cstr, "gl_Vertex"))
					glDisable(GL_VERTEX_ARRAY);
				else if(!strcmp(name_cstr, "gl_Normal"))
					glDisable(GL_NORMAL_ARRAY);
				else if(!strcmp(name_cstr, "gl_Color"))
					glDisable(GL_COLOR_ARRAY);
				else
				{
					int max_texture_units;
					glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &max_texture_units);
					for(int j = 0; j < max_texture_units; ++j)
					{
						if(name == multi_tex_names[j])
						{
							glClientActiveTexture(GL_TEXTURE0 + j);
							glDisable(GL_TEXTURE_COORD_ARRAY);
						}
					}
				}
			}
			else if(ShaderProgram* shader = ShaderProgram::GetActiveProgram())
			{
				GLint index = glGetAttribLocation(shader->program_id, name_cstr);
				if(index != -1)
				{
					glDisableVertexAttribArray((GLuint)index);
					GLDEBUG();
				}
				else
					Debug(((stringstream&)(stringstream() << "Couldn't disable attribute \"" << name << "\"" << endl)).str());
			}				
		}

		glClientActiveTexture(GL_TEXTURE0);			// get texcoords back to working "the normal way"
		glBindBuffer(GL_ARRAY_BUFFER, 0);			// don't leave hardware vbo on
	}

	void VertexBuffer::Draw() { Draw(storage_mode); }

	void VertexBuffer::Draw(DrawMode mode)
	{
		BuildMultiTexNames();

		GLDEBUG();

		PreDrawEnable();
		glDrawArrays((GLenum)mode, 0, num_verts);
		PostDrawDisable();

		GLDEBUG();
	}

	// TODO: implement these
	void VertexBuffer::Draw(DrawMode mode, unsigned int* indices, int num_indices) { }
	void VertexBuffer::Draw(unsigned int num_instances) { }
	void VertexBuffer::Draw(unsigned int num_instances, DrawMode mode) { }
	void VertexBuffer::Draw(unsigned int num_instances, DrawMode mode, unsigned int* indices, int num_indices) { }

	void VertexBuffer::DrawToFeedbackBuffer(VertexBuffer* target, ShaderProgram* shader_program, bool keep_fragments)
	{
		GLDEBUG();

		if(!keep_fragments)
			glEnable(GL_RASTERIZER_DISCARD);

		ShaderProgram::SetActiveProgram(shader_program);

		vector<GLchar const*> strings;
		vector<string> target_attribs = target->GetAttributes();
		for(vector<string>::iterator iter = target_attribs.begin(); iter != target_attribs.end(); ++iter)
			strings.push_back(iter->c_str());

		glTransformFeedbackVaryings(shader_program->program_id, strings.size(), strings.data(), GL_SEPARATE_ATTRIBS);

		// we must re-link the program to force some things to update (i think?)
		glLinkProgram(shader_program->program_id);

		GLuint query;
		glGenQueries(1, &query);

		GLuint out_buf;
		glGenBuffers(1, &out_buf);
		glBindBuffer(GL_ARRAY_BUFFER, out_buf);
		glBufferData(GL_ARRAY_BUFFER, 4 * num_verts * sizeof(float), NULL, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, out_buf);

		glBindVertexArray(target->GetVBO());							// something like glBindVertexArray(transform vertex array name);

		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query); 
		glBeginTransformFeedback((GLenum)storage_mode);

		Draw(storage_mode);

		glEndTransformFeedback();
		glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);
		
		glDisable(GL_RASTERIZER_DISCARD);

		GLuint primitives_written = 0;
		glGetQueryObjectuiv(query, GL_QUERY_RESULT, &primitives_written);

		glDeleteQueries(1, &query);

		/*
		TODO: put results into target
		*/

		GLDEBUG();
	}



	VertexBuffer* VertexBuffer::CreateEmptyCopyAttributes(VertexBuffer* existing)
	{
		VertexBuffer* result = new VertexBuffer(existing->GetStorageMode());

		vector<string> attribs = existing->GetAttributes();
		for(vector<string>::iterator iter = attribs.begin(); iter != attribs.end(); ++iter)
		{
			string& name = *iter;
			result->AddAttribute(name, existing->GetAttribType(name), existing->GetAttribNPerVertex(name));
		}

		return result;
	}
}
