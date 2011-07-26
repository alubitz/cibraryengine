#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace CibraryEngine;

	class DSNLoader;

	class MaterialLoader : public ContentTypeHandler<Material>
	{
		private:

			DSNLoader* dsn_loader;

		public:

			MaterialLoader(ContentMan* man);

			Material* Load(ContentMetadata& what);
			void Unload(Material* content, ContentMetadata& meta);

			int LoadMaterial(string filename, Material** result_out);

	};
}
