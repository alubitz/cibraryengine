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

		/** Positions and colors for up to 12 vertices */
		TerrainVertex verts[12];
		
		/** Indices into the verts and colors arrays; triples are used to make triangles. */
		int indices[16];

		/** Number of unique vertices; -1 indicates cube is out of date */
		int num_vertices;

		/** Initializes a cube triangulation object. Note that the data arrays will not be default-initialized; the cube is initially not valid, so the values in these arrays won't be used anyway */
		CubeTriangles(TerrainChunk* chunk, int x, int y, int z) : chunk(chunk), x(x), y(y), z(z), num_vertices(-1) { }

		/** Call this when one of the 8 nodes at the corners of this cube is changed */
		void Invalidate();

		/** Gets the vertex data for the triangulation of this cube, computing it if it isn't already known; the return value is the number of vertices */
		int GetVertexData(TerrainVertex* verts, int* indices);
	};
}
