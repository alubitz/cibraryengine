#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	class TerrainChunk;

	struct CubeTriangles
	{
		TerrainChunk* chunk;
		int x, y, z;

		/** Whether the vertex data for this CubeTriangles object is up to date */
		bool triangles_valid;
		/** Whether the normal vectors for this CubeTriangles object are up to date */
		bool normals_valid;

		/** Positions for up to 12 vertices; if triangles_valid is false, this must be recomputed */
		Vec3 verts[12];
		
		/** Colors for up to 12 vertices; if triangles_valid is false, this must be recomputed */
		Vec3 colors[12];
		
		/** Normal vectors for up to 12 vertices; if normals_valid is false, this must be recomputed */
		Vec3 normals[12];

		/** Indices into the verts and colors arrays; triples are used to make triangles. If triangles_valid is false, this must be recomputed */
		int indices[16];

		/**
		 * Initializes a cube triangulation object. Note that the data arrays will not be default-initialized; the cube is initially not valid, so the values in these arrays won't be used anyway
		 */
		CubeTriangles(TerrainChunk* chunk, int x, int y, int z);

		/** Call this when one of the 8 nodes at the corners of this cube is changed */
		void InvalidateTriangles();

		/** Call this when a neighboring cube has changed */
		void InvalidateNormals();
	};
}
