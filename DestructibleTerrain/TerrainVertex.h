#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct TerrainVertex
	{
		Vec3 pos;
		Vec4 color;

		TerrainVertex();
		TerrainVertex(Vec3 pos, Vec4 color);
		
		TerrainVertex operator *(float amount);
		TerrainVertex operator +(TerrainVertex a);
	};
}
