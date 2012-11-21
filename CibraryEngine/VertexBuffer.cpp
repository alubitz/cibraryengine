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

		vector<string> attribs;
		GetAttributes(attribs);

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

			map<string, VertexData> nu_attribute_data;
			for(map<string, VertexData>::iterator iter = attribute_data.begin(); iter != attribute_data.end(); ++iter)
			{
				int n_per_vertex = GetAttribNPerVertex(iter->first);
				switch(GetAttribType(iter->first))
				{
					case Float:
					{
						float* new_data = new float[allocated_size * n_per_vertex];
						if(float* old_data = iter->second.floats)
						{
							for(unsigned int i = 0; i < num_verts * n_per_vertex; ++i)
								new_data[i] = old_data[i];
						}
						nu_attribute_data[iter->first] = VertexData(new_data);

						break;
					}
					case Int:
					{
						int* new_data = new int[allocated_size * n_per_vertex];
						if(int* old_data = iter->second.ints)
						{
							for(unsigned int i = 0; i < num_verts * n_per_vertex; ++i)
								new_data[i] = old_data[i];
						}
						nu_attribute_data[iter->first] = VertexData(new_data);

						break;
					}
				}
			}

			for(map<string, VertexData>::iterator iter = attribute_data.begin(); iter != attribute_data.end(); ++iter)
			{
				switch(GetAttribType(iter->first))
				{
					case Float:	{ float* floats = iter->second.floats;	delete[] floats;	break; }
					case Int:	{ int* ints = iter->second.ints;		delete[] ints;		break; }
				}
			}
			attribute_data.clear();

			attribute_data = nu_attribute_data;
		}
		else if(verts == 0)
		{
			allocated_size = 0;

			map<string, VertexData> nu_attribute_data;
			for(map<string, VertexData>::iterator iter = attribute_data.begin(); iter != attribute_data.end(); ++iter)
			{
				switch(GetAttribType(iter->first))
				{
					case Float:	{ float* old_data = iter->second.floats;	if(old_data) { delete[] old_data; } break; }
					case Int:	{ int* old_data = iter->second.ints;		if(old_data) { delete[] old_data; } break; }
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

		switch(type)
		{
			case Float:	{ attribute_data[name].floats =	num_elements > 0 ? new float[num_elements] : NULL;	break; }
			case Int:	{ attribute_data[name].ints =	num_elements > 0 ? new int[num_elements] : NULL;	break; }
		}
	}

	void VertexBuffer::RemoveAttribute(const string& name)
	{
		map<string, VertexAttribute>::iterator found_name = attributes.find(name);
		if(found_name != attributes.end())
		{
			VertexAttribute attribute = found_name->second;
			attributes.erase(found_name);

			map<string, VertexData>::iterator found_attrib = attribute_data.find(name);
			if(found_attrib != attribute_data.end())
			{
				VertexData data = found_attrib->second;

				switch(attribute.type)
				{
					case Float:	{ delete[] data.floats;	break; }
					case Int:	{ delete[] data.ints;	break; }
				}

				attribute_data.erase(found_attrib);
			}
		}
	}

	VertexAttributeType VertexBuffer::GetAttribType(const string& name)
	{
		map<string, VertexAttribute>::iterator found_name = attributes.find(name);
		if(found_name != attributes.end())
			return found_name->second.type;
		else
			return BadVertexAttribute;
	}

	int VertexBuffer::GetAttribNPerVertex(const string& name)
	{
		map<string, VertexAttribute>::iterator found_name = attributes.find(name);
		if(found_name != attributes.end())
			return found_name->second.n_per_vertex;
		else
			return -1;
	}

	void VertexBuffer::GetAttributes(vector<string>& results_out)
	{
		results_out.clear();
		for(map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
			results_out.push_back(iter->first);
	}

	int VertexBuffer::GetVertexSize()
	{
		int total_size = 0;
		for(map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			VertexAttribute& attrib = iter->second;
			switch(attrib.type)
			{
				case Float:	{ total_size += attrib.n_per_vertex * sizeof(float);	break; }
				case Int:	{ total_size += attrib.n_per_vertex * sizeof(int);		break; }
			}
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

	int* VertexBuffer::GetIntPointer(const string& name)
	{
		if(GetAttribType(name) == Int)
			return attribute_data[name].ints;
		else
			return NULL;
	}

	void VertexBuffer::InvalidateVBO()
	{
		if(vbo_id)
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
		for(map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;

			int attrib_size = 0;
			switch(attrib.type)
			{
				case Float:	{ attrib_size = attrib.n_per_vertex * sizeof(float);	break; }
				case Int:	{ attrib_size = attrib.n_per_vertex * sizeof(int);		break; }
			}

			if(attrib_size > 0)
			{
				switch(attrib.type)
				{
					case Float:	{ glBufferSubData(GL_ARRAY_BUFFER, offset * num_verts, attrib_size * num_verts, attribute_data[attrib.name].floats);	break; }
					case Int:	{ glBufferSubData(GL_ARRAY_BUFFER, offset * num_verts, attrib_size * num_verts, attribute_data[attrib.name].ints);		break; }
				}

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
		for(map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;

			int attrib_size = 0;
			switch(attrib.type)
			{
				case Float:	{ attrib_size = attrib.n_per_vertex * sizeof(float);	break; }
				case Int:	{ attrib_size = attrib.n_per_vertex * sizeof(int);		break; }
			}

			if(attrib_size > 0)
			{
				switch(attrib.type)
				{
					case Float:	{ glGetBufferSubData(GL_ARRAY_BUFFER, offset * num_verts, attrib_size * num_verts, attribute_data[attrib.name].floats);	break; }
					case Int:	{ glGetBufferSubData(GL_ARRAY_BUFFER, offset * num_verts, attrib_size * num_verts, attribute_data[attrib.name].ints);	break; }
				}

				offset += attrib_size;
			}
		}

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	// utility stuff... do this once
	string* multi_tex_names = NULL;
	void BuildMultiTexNames()
	{
		if(!multi_tex_names)
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
		for(map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
		{
			const VertexAttribute& attrib = iter->second;
			const string& name = iter->first;
			const char* name_cstr = name.c_str();

			if(name.length() >= 3 && memcmp(name_cstr, "gl_", 3) == 0)
			{
				if(!strcmp(name_cstr, "gl_Vertex"))
				{
					glEnable(GL_VERTEX_ARRAY);
					glVertexPointer(attrib.n_per_vertex, (GLenum)attrib.type, 0, (void*)(num_verts * offset));
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
					glVertexAttribPointer((GLuint)index, attrib.n_per_vertex, attrib.type, false, 0, (void*)(num_verts * offset));

					GLDEBUG();
				}
				else
					Debug(((stringstream&)(stringstream() << "Couldn't enable attribute \"" << name << "\"" << endl)).str());
			}

			switch(attrib.type)
			{
				case Float:	{ offset += attrib.n_per_vertex * sizeof(float);	break; }
				case Int:	{ offset += attrib.n_per_vertex * sizeof(int);		break; }
			}
		}

		GLDEBUG();
	}

	void VertexBuffer::PostDrawDisable()
	{
		GLDEBUG();

		for(map<string, VertexAttribute>::iterator iter = attributes.begin(); iter != attributes.end(); ++iter)
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
				}									// no "else" for disabling (no need for 2x the debug spew)
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

		glDrawArrays((GLenum)mode, 0, num_verts); GLDEBUG();

		PostDrawDisable();
		GLDEBUG();
	}

	// TODO: implement these
	void VertexBuffer::Draw(DrawMode mode, unsigned int* indices, int num_indices) { }
	void VertexBuffer::Draw(unsigned int num_instances) { }
	void VertexBuffer::Draw(unsigned int num_instances, DrawMode mode) { }
	void VertexBuffer::Draw(unsigned int num_instances, DrawMode mode, unsigned int* indices, int num_indices) { }



	VertexBuffer* VertexBuffer::CreateEmptyCopyAttributes(VertexBuffer* existing)
	{
		VertexBuffer* result = new VertexBuffer(existing->GetStorageMode());

		vector<string> attribs;
		existing->GetAttributes(attribs);

		for(vector<string>::iterator iter = attribs.begin(); iter != attribs.end(); ++iter)
		{
			string& name = *iter;
			result->AddAttribute(name, existing->GetAttribType(name), existing->GetAttribNPerVertex(name));
		}

		return result;
	}
}
