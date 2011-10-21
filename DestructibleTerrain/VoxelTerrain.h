#pragma once

#include "StdAfx.h"

#include "Octree.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class VoxelMaterial;

	struct TerrainLeaf
	{
		float value;

		TerrainLeaf();
		TerrainLeaf(float value);
	};

	class VoxelTerrain
	{
		private:

			vector<TerrainLeaf> data;
			int dim[3];

			Vec3 min_xyz;
			Vec3 step_xyz;

			VoxelMaterial* material;
			VertexBuffer* model;

			TerrainLeaf& Element(int x, int y, int z);

		public:

			VoxelTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z);
			~VoxelTerrain();

			void Vis(SceneRenderer* renderer);
	};
}
