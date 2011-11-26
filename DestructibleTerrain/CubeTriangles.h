#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace std;

	class TerrainChunk;

	struct CubeTriangles
	{
		TerrainChunk* chunk;
		int x, y, z;

		bool valid;
		vector<float> data;

		CubeTriangles(TerrainChunk* chunk, int x, int y, int z);

		void Invalidate();
	};
}
