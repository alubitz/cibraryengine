#include "StdAfx.h"
#include "TerrainNode.h"

#include "TerrainChunk.h"

#include "Serialize.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * TerrainNode methods
	 */
	TerrainNode::TerrainNode() : solidity(0), material() { }

	float TerrainNode::GetScalarValue() { return 127.5f - solidity; }
	bool TerrainNode::IsSolid() { return solidity >= 128; }

	unsigned int TerrainNode::Write(ostream& stream)
	{
		WriteByte(solidity, stream);
		
		if(solidity > 0)
			stream.write((char*)(&material), 8);

		return 0;
	}

	unsigned int TerrainNode::Read(istream& stream)
	{
		solidity = ReadByte(stream);

		if(solidity > 0)
			stream.read((char*)(&material), 8);

		return 0;
	}
}
