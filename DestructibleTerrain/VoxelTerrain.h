#pragma once

#include "StdAfx.h"

#include "Octree.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class VoxelTerrain
	{
		public:

			Octree<unsigned char> octree;

			VoxelTerrain();

			void Vis(SceneRenderer* renderer);
	};
}
