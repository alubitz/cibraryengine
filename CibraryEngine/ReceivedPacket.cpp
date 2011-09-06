#include "StdAfx.h"

#include "ReceivedPacket.h"

namespace CibraryEngine
{
	/*
	 * ReceivedPacket methods
	 */
	ReceivedPacket::ReceivedPacket() :
		packet(),
		id(0)
	{
	}

	
	ReceivedPacket::ReceivedPacket(Packet packet, unsigned int id) :
		packet(packet),
		id(id)
	{
	}
}
