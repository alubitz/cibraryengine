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
		valid(false),
		data()
	{
	}

	void CubeTriangles::Invalidate()
	{
		valid = false;
		chunk->InvalidateVBO();
	}
}
