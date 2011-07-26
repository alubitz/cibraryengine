#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class DSNMaterial;

	class DSNLoader
	{
		public:

			ContentMan* content;
			ShaderProgram* shader;
			Texture2D* default_normal;
			Texture2D* default_specular;
			TextureCube* ambient_cubemap;

			DSNLoader(ContentMan* content, ShaderProgram* shader, Texture2D* default_normal, Texture2D* default_specular, TextureCube* ambient_cubemap) :
				content(content),
				shader(shader),
				default_normal(default_normal),
				default_specular(default_specular),
				ambient_cubemap(ambient_cubemap)
			{ }

			DSNMaterial* Load(string asset_name);
	};
}
