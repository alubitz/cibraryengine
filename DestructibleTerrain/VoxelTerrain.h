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
		Vec3 normal;

		TerrainLeaf();
		TerrainLeaf(float value, Vec3 normal);
	};

	class VoxelTerrain
	{
		private:

			vector<TerrainLeaf> data;
			int dim[3];
			int x_span;

			Mat4 scale;
			Mat4 xform;

			VoxelMaterial* material;
			VertexBuffer* model;

			TerrainLeaf& Element(int x, int y, int z);

			VertexBuffer* CreateVBO();

		public:

			VoxelTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z);
			~VoxelTerrain();

			void Vis(SceneRenderer* renderer);
	};
}
