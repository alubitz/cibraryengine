#include "StdAfx.h"
#include "DSNLoader.h"

#include "DSNMaterial.h"

namespace Test
{
	/*
	 * DSNLoader methods
	 */
	DSNMaterial* DSNLoader::Load(string asset_name)
	{
		Texture2D* diffuse = tex_cache->Load(asset_name + "-d");
		Texture2D* specular = tex_cache->Load(asset_name + "-s");
		Texture2D* normal = tex_cache->Load(asset_name + "-n");

		if(specular == NULL)
			specular = default_specular;
		if(normal == NULL)
			normal = default_normal;

		return new DSNMaterial(shader, diffuse, specular, normal);
	}
}
