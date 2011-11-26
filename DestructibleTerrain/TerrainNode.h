#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	class TerrainChunk;

	struct TerrainNode
	{
		unsigned char types[4], weights[4];
		unsigned char solidity;

		TerrainNode();
		TerrainNode(unsigned char type);

		void ClearMaterials();
		unsigned char GetMaterialAmount(unsigned char mat);
		void SetMaterialAmount(unsigned char mat, unsigned char amount);

		float GetScalarValue();
		bool IsSolid();
	};
}
