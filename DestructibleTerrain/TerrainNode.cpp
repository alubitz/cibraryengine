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
	TerrainNode::TerrainNode() : solidity(0), material() { }

	float TerrainNode::GetScalarValue() { return -(solidity - 127.5f); }
	bool TerrainNode::IsSolid() { return GetScalarValue() < 0; }

	unsigned int TerrainNode::Write(ostream& stream)
	{
		WriteByte(solidity, stream);
		
		if(solidity > 0)
			for(int i = 0; i < 4; ++i)
			{
				WriteByte(material.types[i], stream);
				WriteByte(material.weights[i], stream);
			}

		return 0;
	}

	unsigned int TerrainNode::Read(istream& stream)
	{
		if(!stream)
			return 1;

		solidity = ReadByte(stream);

		if(solidity > 0)
			for(int i = 0; i < 4; ++i)
			{
				material.types[i] = ReadByte(stream);
				material.weights[i] = ReadByte(stream);
			}

		return 0;
	}
}
