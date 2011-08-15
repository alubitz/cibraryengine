#pragma once

#include "StdAfx.h"

#include "Content.h"

#include <typeinfo>

namespace CibraryEngine
{
	using namespace std;

	/** Whether a shader is a vertex shader or a fragment shader */
	enum ShaderType
	{
		Vertex,
		Fragment
	};

	/** Class representing a vertex or fragment shader */
	class Shader : public Disposable
	{
		protected:

			void InnerDispose();

		public:

			/** OpenGL id of the shader */
			int shader_id;
			/** Whether it's a vertex shader or fragment shader */
			ShaderType type;
			/** The source code for this shader */
			string source;

			Shader();
			/** Initializes a Shader of the specified type, with the specified source code */
			Shader(ShaderType type, string source);

			/** Compiles the shader */
			void CompileShader();

			/** Creates a fragment shader from the specified file */
			static Shader* FragmentShaderFromFile(string filename);
			/** Creates a vertex shader from the specified file */
			static Shader* VertexShaderFromFile(string filename);
	};

	class ShaderProgram;

	/** Class wrapping interaction with shader variables declared as uniform */
	class UniformVariable
	{
		public:

			/** The name of the uniform variable */
			string name;

			UniformVariable(string name) : name(name) { }
			~UniformVariable() { }

			/** Figures out the location in the ShaderProgram to apply the value, and calls ApplyValue with that */
			void ApplyValue(ShaderProgram* program);
			/** Abstract function where the actual glUniform calls belong */
			virtual void ApplyValue(int location) = 0;

			/** Virtual function to disable any OpenGL states that should't be left on after the shader is done */
			virtual void Disable() { }
	};

	/** A UniformVariable of a specific type */
	template <typename T> class TypedUniformVariable : public UniformVariable
	{
		public:

			TypedUniformVariable(string name) : UniformVariable(name) { }
			virtual ~TypedUniformVariable() { }

			/** Abstract function to get the value of the uniform variable */
			virtual T* GetValue() = 0;
			/** Abstract function to set the value of the uniform variable */
			virtual void SetValue(T* t) = 0;
	};

	struct UTypeInfoComp { bool operator ()(const type_info* L , const type_info* R) { return L->before(*R) != 0; } };

	/** Class representing a linked shader program, which exposes the uniform variables of that shader program */
	class ShaderProgram
	{
		private:

			map<const type_info*, map<string, UniformVariable*>, UTypeInfoComp> type_caches;			// map type_infos to Caches

			void LinkProgram();

			void Build();

		public:

			/** The vertex shader */
			Shader* vertex_shader;
			/** The fragment shader */
			Shader* fragment_shader;
			/** The OpenGL shader program name for this shader */
			int program_id;

			/** Initializess a ShaderProgram given a vertex and fragment shader */
			ShaderProgram(Shader* vertex_shader, Shader* fragment_shader);

			/** Updates all of the uniform variables (so they can call glUniform) */
			void UpdateUniforms();
			/** Disables all of the uniform variables so that no unwanted OpenGL states are left active */
			void DisableUniforms();

			/** Add an exposed uniform variable for this shader */
			template <typename T> void AddUniform(UniformVariable* u) { type_caches[&typeid(T)][u->name] = u; }

			/** Get the value of the specified uniform variable */
			template <typename T> T* GetUniform(string name) { Build(); return (T*)((TypedUniformVariable<T>*)(type_caches[&typeid(T)][name]))->GetValue(); }
			/** Set the value of the specified uniform variable */
			template <typename T> void SetUniform(string name, T* value) { Build(); ((TypedUniformVariable<T>*)(type_caches[&typeid(T)][name]))->SetValue(value); }

			/** Set the active ShaderProgram; pass NULL to use the fixed-function pipeline */
			static void SetActiveProgram(ShaderProgram* program);

			/** Get the active ShaderProgram; returns NULL if the fixed-function pipeline is being used */
			static ShaderProgram* GetActiveProgram();
	};




	struct ShaderLoader : public ContentTypeHandler<Shader>
	{
		ShaderLoader(ContentMan* content) : ContentTypeHandler<Shader>(content) { }
		Shader* Load(ContentMetadata& what);
	};
}
