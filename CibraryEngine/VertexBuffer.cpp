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
		if(vbo_id == 0) 
			BuildVBO(); 

		return vbo_id;
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

	void VertexBuffer::Draw()
	{
		BuildMultiTexNames();

		GLDEBUG();

		unsigned int vbo = GetVBO();


		/*
		 * First, we set everything up for our VBO draw operation...
		 */
		glBindBuffer(GL_ARRAY_BUFFER, vbo);

		int offset = 0;
		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;
			const string& name = iter->first;
			const char* name_cstr = name.c_str();

			if(name.length() >= 3 && memcmp(name_cstr, "gl_", 3) == 0)
			{
				if(name == "gl_Vertex")
				{
					glEnable(GL_VERTEX_ARRAY);
					glVertexPointer(attrib.n_per_vertex, (GLenum)attrib.type, 0,	(void*)(num_verts * offset));
				}
				else if(name == "gl_Normal")
				{
					glEnable(GL_NORMAL_ARRAY);
					glNormalPointer((GLenum)attrib.type, 0, (void*)(num_verts * offset));
				}
				else if(name == "gl_Color")
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
			else
			{
				ShaderProgram* shader = ShaderProgram::GetActiveProgram();
				if(shader != NULL)
				{
					GLuint index = (GLuint)glGetAttribLocation(shader->program_id, name_cstr);
					glEnableVertexAttribArray(index);
					glVertexAttribPointer(index, attrib.n_per_vertex, (GLenum)attrib.type, true, 0, (void*)(num_verts * offset));
				}
			}

			if(attrib.type == Float)
				offset += attrib.n_per_vertex * sizeof(float);
		}


		/*
		 * Now for the draw call itself...
		 */
		glDrawArrays((GLenum)storage_mode, 0, num_verts);


		/*
		 * Now to put everything back the way we found it...
		 */
		for(boost::unordered_map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;
			const string& name = iter->first;
			const char* name_cstr = name.c_str();

			if(name.length() >= 3 && memcmp(name_cstr, "gl_", 3) == 0)
			{
				if(name == "gl_Vertex")
					glDisable(GL_VERTEX_ARRAY);
				else if(name == "gl_Normal")
					glDisable(GL_NORMAL_ARRAY);
				else if(name == "gl_Color")
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
			else
			{
				ShaderProgram* shader = ShaderProgram::GetActiveProgram();
				if(shader != NULL)
				{
					GLuint index = (GLuint)glGetAttribLocation(shader->program_id, name_cstr);
					glDisableVertexAttribArray(index);
				}				
			}
		}

		glClientActiveTexture(GL_TEXTURE0);			// get texcoords back to working "the normal way"
		glBindBuffer(GL_ARRAY_BUFFER, 0);			// don't leave hardware vbo on

		GLDEBUG();
	}

	// TODO: implement these
	void VertexBuffer::Draw(DrawMode mode) { }
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

		GLchar const* strings[] = { "gl_Vertex" };
//		glTransformFeedbackVaryings(shader_program->program_id, 1, strings, GL_SEPARATE_ATTRIBS);

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

//		glBindVertexArray(transform vertex array name);

		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, query); 
		glBeginTransformFeedback((GLenum)storage_mode);
			glDrawArrays((GLenum)storage_mode, 0, num_verts);
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
}
