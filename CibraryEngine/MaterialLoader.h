#pragma once

#include "StdAfx.h"

#include "Content.h"

namespace CibraryEngine
{
	using namespace std;

	class Material;
	class DSNLoader;
	class ShaderProgram;
	class Shader;
	class Texture2D;

	class MaterialLoader : public ContentTypeHandler<Material>
	{
		private:

			DSNLoader* dsn_opaque_loader;
			DSNLoader* dsn_additive_loader;
			DSNLoader* dsn_alpha_loader;

			ShaderProgram* glowy2d_shader;
			ShaderProgram* glowy3d_shader;

			Cache<Shader>* shader_cache;
			Cache<Texture2D>* tex_cache;

		public:

			MaterialLoader(ContentMan* man);

			Material* Load(ContentMetadata& what);
			void Unload(Material* content, ContentMetadata& meta);

			unsigned int LoadMaterial(string filename, Material** result_out);

	};
}
