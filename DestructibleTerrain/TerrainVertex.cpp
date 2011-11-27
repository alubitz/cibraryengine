#include "StdAfx.h"
#include "TerrainVertex.h"

namespace DestructibleTerrain
{
	/* 
	 * TerrainVertex methods
	 */
	TerrainVertex::TerrainVertex() : pos(), color(1.0f, 1.0f, 1.0f, 0.0f) { }
	TerrainVertex::TerrainVertex(Vec3 pos, Vec4 color) : pos(pos), color(color) { }
	
	TerrainVertex TerrainVertex::operator *(float amount) { return TerrainVertex(pos * amount, color * amount); }
	TerrainVertex TerrainVertex::operator +(TerrainVertex a) { return TerrainVertex(pos + a.pos, color + a.color); }
}
