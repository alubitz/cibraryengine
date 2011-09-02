#pragma once

#include "StdAfx.h"

#include "Packet.h"

namespace CibraryEngine
{
	struct ReceivedPacket
	{
		Packet packet;
		unsigned int id;

		/** Default constructor exists mainly to allow these to go directly into STL collections */
		ReceivedPacket();

		ReceivedPacket(Packet packet, unsigned int id);
	};
}
