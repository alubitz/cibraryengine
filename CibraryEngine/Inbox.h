#pragma once

#include "StdAfx.h"

#include "ReceivedPacket.h"

namespace CibraryEngine
{
	using namespace std;

	struct Inbox
	{
		private:

			list<ReceivedPacket> collection;

			string to_be_assigned;
			unsigned int next_packet;

		public:

			Inbox();

			list<ReceivedPacket> Receive(unsigned char* incoming, unsigned int len);

			list<ReceivedPacket> GetPackets();
			void ClearPackets();
	};
}
