#include "StdAfx.h"
#include "TerrainVertex.h"

namespace CibraryEngine
{
	/* 
	 * TerrainVertex methods
	 */
	TerrainVertex::TerrainVertex() : pos(), normal(), normal_valid(false) { }
	TerrainVertex::TerrainVertex(Vec3 pos, MultiMaterial material) : pos(pos), normal(), normal_valid(false), material(material) { }
}
