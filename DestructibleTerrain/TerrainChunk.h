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

			int chunk_x, chunk_y, chunk_z;

			Mat4 xform;

			VoxelMaterial* material;
			VertexBuffer* model;

			VertexBuffer* CreateVBO();
			void InvalidateVBO();

			VoxelTerrain* owner;

		public:

			static const unsigned int ChunkSize = 8;
			static const unsigned int ChunkSizeSquared = 64;

			TerrainChunk(VoxelMaterial* material, VoxelTerrain* owner, int x, int y, int z);
			~TerrainChunk();

			/** 
			 * Get a reference to the specified element
			 */
			TerrainLeaf& Element(int x, int y, int z);
			
			/**
			 * Get a pointer to the element at the specified position relative to this chunk, or NULL if the position is not within a non-NULL TerrainChunk
			 * Unlike Element, this works for elements outside the range of this chunk
			 */
			TerrainLeaf* GetElementRelative(int x, int y, int z);

			void Solidify();
			void Explode(Vec3 center, float blast_force);

			void Vis(SceneRenderer* renderer, Mat4 main_xform);

			template <class T> void PopulateValues(T t);
	};

	template <class T> void TerrainChunk::PopulateValues(T t) 
	{
		for(int x = 0; x < ChunkSize; x++)
			for(int y = 0; y < ChunkSize; y++)
				for(int z = 0; z < ChunkSize; z++)
					Element(x, y, z) = t(x, y, z);
	}
}
