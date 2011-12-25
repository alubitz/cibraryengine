#include "StdAfx.h"

#include "Outbox.h"
#include "Packet.h"

namespace CibraryEngine
{
	/*
	 * Outbox methods
	 */
	Outbox::Outbox() : packets() { }

	string Outbox::GetBytesAndClear()
	{
		// The C# version of this function had everything enclosed in		lock(packets) {		}

		string result;

		for(list<Packet>::iterator iter = packets.begin(); iter != packets.end(); ++iter)
			result += iter->GetContentBytes();

		return result;
	}
}
