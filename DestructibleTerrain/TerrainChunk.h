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
			bool vbo_valid;

			VertexBuffer* CreateVBO();

			VoxelTerrain* owner;

		public:

			// If these were unsigned ints it would cause some stupid errors with underflow
			static const int ChunkSize = 8;
			static const int ChunkSizeSquared = ChunkSize * ChunkSize;



			TerrainChunk(VoxelMaterial* material, VoxelTerrain* owner, int x, int y, int z);
			~TerrainChunk();

			template <class T> void PopulateValues(T t);



			/** 
			 * Get a pointer to the specified node
			 */
			TerrainNode* GetNode(int x, int y, int z);
			CubeTriangles* GetCube(int x, int y, int z);
			
			/**
			 * Get a pointer to the node at the specified position relative to this chunk, or NULL if the position is not within a non-NULL TerrainChunk
			 * Unlike GetNode, this works for nodes outside the range of this chunk
			 */
			TerrainNode* GetNodeRelative(int x, int y, int z);
			CubeTriangles* GetCubeRelative(int x, int y, int z);

			bool GetRelativePositionInfo(int x, int y, int z, TerrainChunk*& chunk, int& dx, int &dy, int& dz);						// internal utility function, but possibly useful for external code as well (thus, public)

			void InvalidateNode(int x, int y, int z);
			void InvalidateCubeRelative(int x, int y, int z);
			void InvalidateVBO();

			void Solidify();
			void Explode(Vec3 blast_center, float inner_radius, float outer_radius);

			void Vis(SceneRenderer* renderer, Mat4 main_xform);

			// Member I/O functions
			unsigned int Write(ostream& stream);
			unsigned int Read(istream& stream);
	};

	template <class T> void TerrainChunk::PopulateValues(T t) 
	{
		for(int x = 0; x < ChunkSize; x++)
			for(int y = 0; y < ChunkSize; y++)
				for(int z = 0; z < ChunkSize; z++)
					*GetNode(x, y, z) = t(x, y, z);
	}
}
