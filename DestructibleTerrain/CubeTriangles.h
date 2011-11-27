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

		bool valid;

		/**
		 * Initializes a cube triangulation object. Note that the data arrays will not be default-initialized; the cube is initially not valid, so the values in these arrays won't be used anyway
		 */
		CubeTriangles(TerrainChunk* chunk, int x, int y, int z);

		/** Call this when one of the 8 nodes at the corners of this cube is changed */
		void Invalidate();

		/** Gets the vertex data for the triangulation of this cube, computing it if it isn't already known */
		void AppendVertexData(vector<TerrainVertex>& target);
	};
}
