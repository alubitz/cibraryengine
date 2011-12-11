#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct TerrainVertex
	{
		Vec3 pos;

		TerrainVertex();
		TerrainVertex(Vec3 pos);
		
		TerrainVertex operator *(float amount);
		TerrainVertex operator +(TerrainVertex a);
	};
}
