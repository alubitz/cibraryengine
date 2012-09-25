#include "StdAfx.h"
#include "Shader.h"

#include "Serialize.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * Shader methods
	 */
	Shader::Shader() : shader_id(0), source() { }
	Shader::Shader(ShaderType type, const string& source) :
		shader_id(0),
		type(type),
		source(source)
	{
	}

	void Shader::InnerDispose() { glDeleteShader(shader_id); shader_id = 0; }

	void Shader::CompileShader()
	{
		if(shader_id == 0)
		{
			shader_id = glCreateShader(type == Vertex ? GL_VERTEX_SHADER : GL_FRAGMENT_SHADER);

			const char* source_string = source.c_str();
			glShaderSource(shader_id, 1, &source_string, NULL);
			glCompileShader(shader_id);
		}
	}

	Shader* Shader::FragmentShaderFromFile(const string& filename)
	{
		string str;
		if(GetFileString("Files/Shaders/" + filename + ".txt", &str) != 0)
			return NULL;
		return new Shader(Fragment, str);
	}

	Shader* Shader::VertexShaderFromFile(const string& filename)
	{
		string str;
		if(GetFileString("Files/Shaders/" + filename + ".txt", &str) != 0)
			return NULL;
		return new Shader(Vertex, str);
	}




	/*
	 * UniformVariable methods
	 */
	void UniformVariable::ApplyValue(ShaderProgram* program)
	{
		int program_id = program->program_id;
		int location = glGetUniformLocation(program_id, name.c_str());

		if(location == -1)
			Debug(((stringstream&)(stringstream() << "Uniform variable \"" << name << "\" not present or not used in shader!" << endl)).str());

		ApplyValue(location);			// should this be an "else" ?
	}




	/*
	 * ShaderProgram methods
	 */
	ShaderProgram::ShaderProgram(Shader* vertex_shader, Shader* fragment_shader) :
		vertex_shader(vertex_shader),
		fragment_shader(fragment_shader),
		program_id(0)
	{
	}

	void ShaderProgram::InnerDispose()
	{
		for(map<const type_info*, boost::unordered_map<string, UniformVariable*>, UTypeInfoComp>::iterator iter = type_caches.begin(); iter != type_caches.end(); ++iter)
		{
			boost::unordered_map<string, UniformVariable*>& m = iter->second;
			for(boost::unordered_map<string, UniformVariable*>::iterator jter = m.begin(); jter != m.end(); ++jter)
				delete jter->second;

			m.clear();
		}
		type_caches.clear();

		glDeleteProgram(program_id);
		program_id = 0;

		// not deleting these because they are a separate Content item and may still be used elsewhere
		vertex_shader = NULL;
		fragment_shader = NULL;
	}

	void ShaderProgram::Build()
	{
		if(program_id != 0)
			return;

		if(vertex_shader != NULL && fragment_shader != NULL)
		{
			GLDEBUG();
			program_id = glCreateProgram();

			GLDEBUG();

			LinkProgram();

			char vlog[1024];
			char flog[1024];
			char plog[1024];
			glGetShaderInfoLog(vertex_shader->shader_id, 1024, NULL, vlog);
			glGetShaderInfoLog(fragment_shader->shader_id, 1024, NULL, flog);
			glGetProgramInfoLog(program_id, 1024, NULL, plog);

			int vertex_status, fragment_status, program_status;

			glGetShaderiv(vertex_shader->shader_id, GL_COMPILE_STATUS, &vertex_status);
			glGetShaderiv(fragment_shader->shader_id, GL_COMPILE_STATUS, &fragment_status);
			glGetProgramiv(program_id, GL_LINK_STATUS, &program_status);

			GLDEBUG();

			if(vertex_status == 0)
			{
				Debug(vlog);
				program_id = 0;
			}
			if(fragment_status == 0)
			{
				Debug(flog);
				program_id = 0;
			}
			if(program_status == 0)
			{
				Debug(plog);
				program_id = 0;
			}

			GLDEBUG();
		}
	}

	void ShaderProgram::LinkProgram()
	{
		vertex_shader->CompileShader();
		fragment_shader->CompileShader();
		glAttachShader(program_id, vertex_shader->shader_id);
		glAttachShader(program_id, fragment_shader->shader_id);
		glLinkProgram(program_id);
	}

	void ShaderProgram::UpdateUniforms()
	{
		Build();

		for(map<const type_info*, boost::unordered_map<string, UniformVariable*>, UTypeInfoComp>::iterator iter = type_caches.begin(); iter != type_caches.end(); ++iter)
		{
			boost::unordered_map<string, UniformVariable*>& m = iter->second;
			for(boost::unordered_map<string, UniformVariable*>::iterator jter = m.begin(); jter != m.end(); ++jter)
				jter->second->ApplyValue(this);
		}
	}

	void ShaderProgram::DisableUniforms()
	{
		Build();

		for(map<const type_info*, boost::unordered_map<string, UniformVariable*>, UTypeInfoComp>::iterator iter = type_caches.begin(); iter != type_caches.end(); ++iter)
		{
			boost::unordered_map<string, UniformVariable*>& m = iter->second;
			for(boost::unordered_map<string, UniformVariable*>::iterator jter = m.begin(); jter != m.end(); ++jter)
				jter->second->Disable();
		}
	}

	ShaderProgram* active_program = NULL;
	void ShaderProgram::SetActiveProgram(ShaderProgram* program)
	{
		GLDEBUG();

		if(active_program != NULL && program != active_program)
		{
			active_program->DisableUniforms();
			GLDEBUG();
		}

		if(program != NULL && program->program_id == 0)
			program->Build();

		if(program != NULL && program->program_id != 0)
		{
			GLDEBUG();

			glUseProgram(program->program_id);

			GLDEBUG();

			program->UpdateUniforms();

			GLDEBUG();

			active_program = program;
		}
		else
		{
			glUseProgram(0);

			int max;
			glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &max);
			for(int i = 0; i < max; ++i)
			{
				glActiveTexture(GL_TEXTURE0 + i);
				glDisable(GL_TEXTURE_2D);
			}

			glActiveTexture(GL_TEXTURE0);
			glEnable(GL_TEXTURE_2D);

			active_program = NULL;

			GLDEBUG();
		}
	}

	ShaderProgram* ShaderProgram::GetActiveProgram()
	{
		return active_program;
	}




	/*
	 * ShaderLoader methods
	 */
	Shader* ShaderLoader::Load(ContentMetadata& what)
	{
		const string& asset_name = what.name;
		if(asset_name.rfind("-f") == asset_name.length() - 2)
			return Shader::FragmentShaderFromFile(asset_name);
		else
			return Shader::VertexShaderFromFile(asset_name);
	}
}
