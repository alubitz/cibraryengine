#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct Packet;

	struct Outbox
	{
		list<Packet> packets;

		Outbox();

		vector<unsigned char> GetBytesAndClear();
	};
}
