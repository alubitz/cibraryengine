#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class DSNLoader;

	class MaterialLoader : public ContentTypeHandler<Material>
	{
		private:

			DSNLoader* dsn_loader;
			ShaderProgram* glowy2d_shader;
			ShaderProgram* glowy3d_shader;

		public:

			MaterialLoader(ContentMan* man);

			Material* Load(ContentMetadata& what);
			void Unload(Material* content, ContentMetadata& meta);

			int LoadMaterial(string filename, Material** result_out);

	};
}
