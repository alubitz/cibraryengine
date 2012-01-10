#pragma once

#include "StdAfx.h"
#include "MultiMaterial.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct TerrainVertex
	{
		Vec3 pos;
		MultiMaterial material;

		TerrainVertex();
		TerrainVertex(Vec3 pos, MultiMaterial material);
	};
}
