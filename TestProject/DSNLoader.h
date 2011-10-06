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
			BlendStyle default_blend_style;

			DSNLoader(ContentMan* content, ShaderProgram* shader, Texture2D* default_normal, Texture2D* default_specular, BlendStyle default_blend_style) :
				content(content),
				tex_cache(content->GetCache<Texture2D>()),
				shader(shader),
				default_normal(default_normal),
				default_specular(default_specular),
				default_blend_style(default_blend_style)
			{ }

			DSNMaterial* Load(string asset_name);
	};
}
