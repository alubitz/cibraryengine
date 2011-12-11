#include "StdAfx.h"
#include "TerrainVertex.h"

namespace DestructibleTerrain
{
	/* 
	 * TerrainVertex methods
	 */
	TerrainVertex::TerrainVertex() : pos() { }
	TerrainVertex::TerrainVertex(Vec3 pos) : pos(pos) { }
	
	TerrainVertex TerrainVertex::operator *(float amount) { return TerrainVertex(pos * amount); }
	TerrainVertex TerrainVertex::operator +(TerrainVertex a) { return TerrainVertex(pos + a.pos); }
}
