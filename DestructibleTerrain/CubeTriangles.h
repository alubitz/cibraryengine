#pragma once

#include "StdAfx.h"

#include "TerrainVertex.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	class TerrainChunk;

	struct CubeTriangles
	{
		TerrainChunk* chunk;
		int x, y, z;

		struct CacheData
		{
			/** Positions and colors for up to 12 vertices */
			TerrainVertex verts[12];
			
			/** Indices into the verts and colors arrays; triples are used to make triangles. */
			char indices[16];
		};
		CacheData* cache;				// this can be NULL without recomputation being necessary!

		/** Number of unique vertices; -1 indicates cube is out of date */
		char num_vertices;

		/** Initializes a cube triangulation object. Note that the data arrays will not be default-initialized; the cube is initially not valid, so the values in these arrays won't be used anyway */
		CubeTriangles(TerrainChunk* chunk, int x, int y, int z) : chunk(chunk), x(x), y(y), z(z), cache(NULL), num_vertices(-1) { }
		~CubeTriangles() { if(cache != NULL) { delete cache; cache = NULL; } }

		/** Call this when one of the 8 nodes at the corners of this cube is changed */
		void Invalidate();

		void BuildAsNeeded();
	};
}
