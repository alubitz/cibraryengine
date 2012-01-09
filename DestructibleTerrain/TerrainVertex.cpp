#include "StdAfx.h"
#include "TerrainVertex.h"

namespace DestructibleTerrain
{
	/* 
	 * TerrainVertex methods
	 */
	TerrainVertex::TerrainVertex() : pos() { }
	TerrainVertex::TerrainVertex(Vec3 pos, MultiMaterial material) : pos(pos), material(material) { }
	
	TerrainVertex TerrainVertex::operator *(float amount) { return TerrainVertex(pos * amount, material * amount); }
	TerrainVertex TerrainVertex::operator +(TerrainVertex a) { return TerrainVertex(pos + a.pos, material + a.material); }
}
