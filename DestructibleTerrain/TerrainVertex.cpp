#include "StdAfx.h"
#include "TerrainVertex.h"

namespace DestructibleTerrain
{
	/* 
	 * TerrainVertex methods
	 */
	TerrainVertex::TerrainVertex() : pos() { }
	TerrainVertex::TerrainVertex(Vec3 pos, MultiMaterial material) : pos(pos), material(material) { }
}
