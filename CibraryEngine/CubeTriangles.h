#pragma once

#include "StdAfx.h"

#include "TerrainVertex.h"

namespace DestructibleTerrain
{
	class TerrainChunk;

	struct CubeTriangles
	{
		int lod;

		TerrainChunk* chunk;
		int x, y, z;

		struct CacheData
		{
			TerrainVertex verts[3];
			
			/** Indices into the vertex array; triples are used to make triangles. */
			char indices[16];
		};
		CacheData* cache;				// this can be NULL without recomputation being necessary!

		/** Number of unique vertices; -1 indicates cube is out of date */
		char num_vertices;

		/** Initializes a cube triangulation object. Note that the data arrays will not be default-initialized; the cube is initially not valid, so the values in these arrays won't be used anyway */
		CubeTriangles(TerrainChunk* chunk, int lod, int x, int y, int z) : lod(lod), chunk(chunk), x(x), y(y), z(z), cache(NULL), num_vertices(-1) { }
		~CubeTriangles() { if(cache != NULL) { delete cache; cache = NULL; } }

		/** Call this when one of the 8 nodes at the corners of this cube is changed */
		void Invalidate();

		void InvalidateNormals();

		void BuildAsNeeded();
	};
}
