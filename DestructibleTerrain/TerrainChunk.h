#pragma once

#include "StdAfx.h"

#include "VoxelTerrain.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	class VoxelTerrain;

	class TerrainChunk
	{
		private:

			vector<TerrainLeaf> data;
			int dim[3];
			int x_span;

			int chunk_x, chunk_y, chunk_z;

			Mat4 xform;

			VoxelMaterial* material;
			VertexBuffer* model;

			VertexBuffer* CreateVBO();
			void InvalidateVBO();

			VoxelTerrain* owner;

		public:

			TerrainChunk(VoxelMaterial* material, VoxelTerrain* owner, int x, int y, int z);
			~TerrainChunk();

			TerrainLeaf& Element(int x, int y, int z);
			void GetDimensions(int& x, int& y, int& z);

			void Solidify();

			void Explode(Vec3 center, float blast_force);
			void Erode();

			void Vis(SceneRenderer* renderer, Mat4 main_xform);

			template <class T> void PopulateValues(T t) 
			{
				for(int x = 0; x < dim[0]; x++)
					for(int y = 0; y < dim[1]; y++)
						for(int z = 0; z < dim[2]; z++)
							Element(x, y, z) = t(x, y, z);
			}
	};
}
