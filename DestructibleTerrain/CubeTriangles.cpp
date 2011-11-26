#include "StdAfx.h"
#include "CubeTriangles.h"

#include "TerrainChunk.h"

namespace DestructibleTerrain
{
	/*
	 * CubeTriangles methods
	 */
	CubeTriangles::CubeTriangles(TerrainChunk* chunk, int x, int y, int z) :
		chunk(chunk),
		x(x),
		y(y),
		z(z),
		triangles_valid(false),
		normals_valid(false)
	{
	}

	void CubeTriangles::InvalidateTriangles()
	{
		triangles_valid = false;
		normals_valid = false;
		chunk->InvalidateVBO();

		// TODO: invalidate neighboring cubes' normals?
	}

	void CubeTriangles::InvalidateNormals() 
	{
		normals_valid = false;
		chunk->InvalidateVBO();
	}
}
