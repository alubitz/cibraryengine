#pragma once

#include "StdAfx.h"

#include "MultiMaterial.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class TerrainChunk;

	struct TerrainNode
	{
		unsigned char solidity;
		MultiMaterial material;
		
		TerrainNode();

		float GetScalarValue();
		bool IsSolid();

		unsigned int Write(ostream& stream);
		unsigned int Read(istream& stream);
	};
}
