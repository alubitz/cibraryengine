#include "StdAfx.h"
#include "TerrainNode.h"

#include "TerrainChunk.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	/*
	 * TerrainNode methods
	 */
	TerrainNode::TerrainNode() : solidity(0) { ClearMaterials(); }

	float TerrainNode::GetScalarValue() { return -(solidity - 127.5f); }
	bool TerrainNode::IsSolid() { return GetScalarValue() < 0; }

	void TerrainNode::ClearMaterials()
	{ 
		types[0] = types[1] = types[2] = types[3] = 0;
		weights[0] = 255;
		weights[1] = weights[2] = weights[3] = 0; 
	}

	unsigned char TerrainNode::GetMaterialAmount(unsigned char mat)
	{
		int total = 0;

		for(int i = 0; i < 4; i++)
			if(types[i] == mat)
				total += weights[i];
		
		return (unsigned char)max(0, min(255, total));
	}

	void TerrainNode::SetMaterialAmount(unsigned char mat, unsigned char amount)
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

	Vec4 TerrainNode::GetColor() 
	{
		/*
		Vec4 color;

		for(int i = 0; i < 4; i++)
		{
			switch(types[i])
			{
			case 1:
			
				// stone color
				color += Vec4(0.5f, 0.5f, 0.5f, 1.0f) * weights[i];
				break;

			case 2:
			
				// dirt (sand) color
				color += Vec4(0.85f, 0.75f, 0.55f, 1.0f) * weights[i];
				break;
			}
		}

		if(color.w != 0)
			color /= color.w;

		return color;
		*/
		return Vec4(1.0f, 1.0f, 1.0f, 1.0f);
	}




	unsigned int TerrainNode::Write(ostream& stream)
	{
		WriteByte(solidity, stream);
		for(int i = 0; i < 4; i++)
		{
			WriteByte(types[i], stream);
			WriteByte(weights[i], stream);
		}
		return 0;
	}

	unsigned int TerrainNode::Read(istream& stream)
	{
		solidity = ReadByte(stream);
		for(int i = 0; i < 4; i++)
		{
			types[i] = ReadByte(stream);
			weights[i] = ReadByte(stream);
		}
		return 0;
	}
}
