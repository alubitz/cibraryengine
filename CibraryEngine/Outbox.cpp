#include "StdAfx.h"

#include "Outbox.h"
#include "Packet.h"

namespace CibraryEngine
{
	/*
	 * Outbox methods
	 */
	Outbox::Outbox() : packets() { }

	vector<unsigned char> Outbox::GetBytesAndClear()
	{
		// The C# version of this function had everything enclosed in		lock(packets) {		}

		vector<unsigned char> result;

		for(list<Packet>::iterator iter = packets.begin(); iter != packets.end(); iter++)
		{
			unsigned int len = iter->GetContentLength();
			vector<unsigned char> packet_bytes = iter->GetContentBytes();
			
			unsigned int start = result.size();
			result.resize(start + len);

			for(unsigned int i = 0; i < len && i < packet_bytes.size(); i++)
				result[i + start] = packet_bytes[i];
		}

		return result;
	}
}
