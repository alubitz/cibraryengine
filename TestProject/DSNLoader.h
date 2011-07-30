#pragma once

#include "StdAfx.h"

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

			DSNLoader(ContentMan* content, ShaderProgram* shader, Texture2D* default_normal, Texture2D* default_specular) :
				content(content),
				shader(shader),
				default_normal(default_normal),
				default_specular(default_specular)
			{ }

			DSNMaterial* Load(string asset_name);
	};
}
