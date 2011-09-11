#include "StdAfx.h"

#include "Inbox.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * Inbox methods
	 */
	Inbox::Inbox() :
		collection(),
		to_be_assigned(),
		next_packet(1)
	{
	}

	list<ReceivedPacket> Inbox::Receive(unsigned char* incoming, unsigned int len)
	{
		if(len > 0)
		{
			for(unsigned int i = 0; i < len; i++)
				to_be_assigned.push_back(incoming[i]);
                
            list<ReceivedPacket> newly_received;
            while (true)         // not really infinite, see below
            {
				Packet packet;

                if (!Packet::MaybeExtractPacket(to_be_assigned, to_be_assigned, packet))
                    break;
                else
                {
                    ReceivedPacket received_packet(packet, next_packet++);
                    collection.push_back(received_packet);
                    newly_received.push_back(received_packet);
                }
            }

            return newly_received;
        }
        else
            return list<ReceivedPacket>();
	}

	list<ReceivedPacket> Inbox::GetPackets()
	{
		list<ReceivedPacket> result;
		for(list<ReceivedPacket>::iterator iter = collection.begin(); iter != collection.end(); iter++)
			result.push_back(*iter);

		return result;
	}

	void Inbox::ClearPackets() { collection.clear(); }
}
