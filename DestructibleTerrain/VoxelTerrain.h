#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class VoxelMaterial;
	
	struct TerrainNode;
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
			bool PosToNode(Vec3 pos, TerrainChunk*& chunk, int& x, int& y, int& z);

			int GetXDim() { return dim[0]; }
			int GetYDim() { return dim[1]; }
			int GetZDim() { return dim[2]; }
			Mat4 GetTransform() { return scale * xform; }

			void Vis(SceneRenderer* renderer);

			void Solidify();
			void Explode(Vec3 center, float inner_radius, float outer_radius);
	};
}
