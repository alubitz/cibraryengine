#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	class TerrainChunk;

	struct TerrainLeaf
	{
		unsigned char types[4], weights[4];
		unsigned char solidity;

		TerrainLeaf();
		TerrainLeaf(unsigned char type);

		void ClearMaterials();
		unsigned char GetMaterialAmount(unsigned char mat);
		void SetMaterialAmount(unsigned char mat, unsigned char amount);
		int GetTotalNonzero();

		float GetScalarValue();
		bool IsSolid();
	};

	struct TerrainLeafReference
	{
		TerrainChunk* chunk;
		int x, y, z;

		TerrainLeafReference();
		TerrainLeafReference(TerrainChunk* chunk, int x, int y, int z);

		operator TerrainLeaf&();

		void operator =(TerrainLeaf& leaf);
	};
}
