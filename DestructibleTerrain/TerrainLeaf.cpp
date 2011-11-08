#include "StdAfx.h"
#include "TerrainLeaf.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	/*
	 * TerrainLeaf methods
	 */
	TerrainLeaf::TerrainLeaf() : solidity(0) { ClearMaterials(); }
	TerrainLeaf::TerrainLeaf(unsigned char type) : solidity(255)
	{
		types[0] = type;
		types[1] = types[2] = types[3] = 0;
		
		weights[0] = 255;
		weights[1] = weights[2] = weights[3] = 0;
	}

	void TerrainLeaf::ClearMaterials()
	{ 
		types[0] = types[1] = types[2] = types[3] = 0;
		weights[0] = 255;
		weights[1] = weights[2] = weights[3] = 0; 
	}

	unsigned char TerrainLeaf::GetMaterialAmount(unsigned char mat)
	{
		int total = 0;

		for(int i = 0; i < 4; i++)
			if(types[i] == mat)
				total += weights[i];
		
		return (unsigned char)max(0, min(255, total));
	}

	void TerrainLeaf::SetMaterialAmount(unsigned char mat, unsigned char amount)
	{
		int cur = GetMaterialAmount(mat);
		if(cur < amount)
		{
			// increasing the amount
			if(cur > 0)
			{
				// there is already a slot for this material
				for(int i = 1; i < 4; i++)
					if(types[i] == mat)
						weights[i] = amount;
			}
			else
			{
				// scrap whatever slot has the least stuff in it
				int min = 0;
				for(int i = 1; i < 4; i++)
					if(weights[i] < weights[min])
						min = i;
				types[min] = mat;
				weights[min] = amount;
			}
		}
		else if(cur > amount)
		{
			// decreasing the amount
			for(int i = 0; i < 4; i++)
				if(types[i] == mat)
					weights[i] = amount;
		}
	}
	int TerrainLeaf::GetTotalNonzero()
	{
		int total = 0;
		for(int i = 0; i < 4; i++)
			if(types[i] != 0)
				total += weights[i];
		return total;
	}

	float TerrainLeaf::GetScalarValue() { return -(solidity - 127.5f); }

	bool TerrainLeaf::IsSolid() { return GetScalarValue() < 0; }
}
