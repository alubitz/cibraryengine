#pragma once

#include "StdAfx.h"

#include "VoxelTerrain.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	class VoxelTerrain;
	struct CubeTriangles;

	class TerrainChunk
	{
		private:

			vector<TerrainNode> node_data;
			vector<CubeTriangles> tri_data;

			int chunk_x, chunk_y, chunk_z;

			Mat4 xform;

			VoxelMaterial* material;
			VertexBuffer* model;

			VertexBuffer* CreateVBO();

			VoxelTerrain* owner;

		public:

			// If these were unsigned ints it would cause some stupid errors with underflow
			static const int ChunkSize = 16;
			static const int ChunkSizeSquared = ChunkSize * ChunkSize;

			TerrainChunk(VoxelMaterial* material, VoxelTerrain* owner, int x, int y, int z);
			~TerrainChunk();

			/** 
			 * Get a pointer to the specified node
			 */
			TerrainNode* GetNode(int x, int y, int z);
			
			/**
			 * Get a pointer to the node at the specified position relative to this chunk, or NULL if the position is not within a non-NULL TerrainChunk
			 * Unlike GetNode, this works for nodes outside the range of this chunk
			 */
			TerrainNode* GetNodeRelative(int x, int y, int z);

			CubeTriangles* GetCube(int x, int y, int z);
			CubeTriangles* GetCubeRelative(int x, int y, int z);

			void InvalidateVBO();

			void Solidify();
			void TerrainChunk::Explode(Vec3 blast_center, float blast_force, set<TerrainChunk*>& affected_chunks);

			void Vis(SceneRenderer* renderer, Mat4 main_xform);

			template <class T> void PopulateValues(T t);
	};

	template <class T> void TerrainChunk::PopulateValues(T t) 
	{
		for(int x = 0; x < ChunkSize; x++)
			for(int y = 0; y < ChunkSize; y++)
				for(int z = 0; z < ChunkSize; z++)
					*GetNode(x, y, z) = t(x, y, z);
	}
}
