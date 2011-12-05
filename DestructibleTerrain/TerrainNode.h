#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class TerrainChunk;

	struct TerrainNode
	{
		unsigned char solidity;
		unsigned char types[4], weights[4];

		TerrainNode();

		float GetScalarValue();
		bool IsSolid();
		
		void ClearMaterials();
		unsigned char GetMaterialAmount(unsigned char mat);
		void SetMaterialAmount(unsigned char mat, unsigned char amount);
		
		Vec4 GetColor();

		unsigned int Write(ostream& stream);
		unsigned int Read(istream& stream);
	};
}
