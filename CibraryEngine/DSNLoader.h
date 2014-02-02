#pragma once

#include "StdAfx.h"

#include "Content.h"

namespace CibraryEngine
{
	using namespace std;

	class Texture2D;
	class ShaderProgram;

	enum BlendStyle;
	class DSNMaterial;

	class DSNLoader
	{
		private:

			ContentMan* content;
			Cache<Texture2D>* tex_cache;

		public:

			ShaderProgram* shader;
			ShaderProgram* shadow_shader;
			Texture2D* default_normal;
			Texture2D* default_specular;
			BlendStyle default_blend_style;

			DSNLoader(ContentMan* content, ShaderProgram* shader, ShaderProgram* shadow_shader, Texture2D* default_normal, Texture2D* default_specular, BlendStyle default_blend_style) :
				content(content),
				tex_cache(content->GetCache<Texture2D>()),
				shader(shader),
				shadow_shader(shadow_shader),
				default_normal(default_normal),
				default_specular(default_specular),
				default_blend_style(default_blend_style)
			{ }

			DSNMaterial* Load(const string& asset_name);
	};
}
