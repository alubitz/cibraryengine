#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct ReceivedPacket;

	struct Inbox
	{
		private:

			list<ReceivedPacket> collection;

			unsigned char* to_be_assigned;
			unsigned int next_packet;

		public:

			Inbox();

			list<ReceivedPacket> Receive(unsigned char* incoming, unsigned int len);

			list<ReceivedPacket> GetPackets();
			void ClearPackets();
	};
}
