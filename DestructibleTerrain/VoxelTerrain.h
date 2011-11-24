#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class VoxelMaterial;
	
	struct TerrainLeaf;
	struct TerrainLeafReference;

	class TerrainChunk;

	class VoxelTerrain
	{
		private:

			vector<TerrainChunk*> chunks;
			int dim[3];
			int x_span;

			Mat4 scale;
			Mat4 xform;

			VoxelMaterial* material;

		public:

			VoxelTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z);
			~VoxelTerrain();

			TerrainChunk* Chunk(int x, int y, int z);
			TerrainLeafReference GetLeafReference(int x, int y, int z);

			void GetDimensions(int& x, int& y, int& z);


			void Vis(SceneRenderer* renderer);

			void Explode();
	};
}
