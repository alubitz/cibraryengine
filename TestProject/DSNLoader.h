#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class DSNMaterial;

	class DSNLoader
	{
		private:

			ContentMan* content;
			Cache<Texture2D>* tex_cache;

		public:

			ShaderProgram* shader;
			Texture2D* default_normal;
			Texture2D* default_specular;

			DSNLoader(ContentMan* content, ShaderProgram* shader, Texture2D* default_normal, Texture2D* default_specular) :
				content(content),
				tex_cache(content->GetCache<Texture2D>()),
				shader(shader),
				default_normal(default_normal),
				default_specular(default_specular)
			{ }

			DSNMaterial* Load(string asset_name);
	};
}
