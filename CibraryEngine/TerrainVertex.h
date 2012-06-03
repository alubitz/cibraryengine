#pragma once

#include "StdAfx.h"
#include "MultiMaterial.h"

#include "Vector.h"

namespace CibraryEngine
{
	struct TerrainVertex
	{
		Vec3 pos;

		Vec3 normal;
		bool normal_valid;
		
		MultiMaterial material;

		TerrainVertex();
		TerrainVertex(Vec3 pos, MultiMaterial material);
	};
}
