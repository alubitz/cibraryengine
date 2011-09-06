#include "StdAfx.h"

#include "Connection.h"
#include "Packet.h"

#include "Inbox.h"
#include "Outbox.h"

namespace CibraryEngine
{
	/*
	 * Connection methods
	 */
	Connection::Connection() : Disposable(), inbox(), outbox() { }

	void Connection::InnerDispose()
	{
		if(IsConnected())
			Disconnect();
	}

	void Connection::BufferedSend(Packet p) { outbox.packets.push_back(p); }

	void Connection::SendBufferedPackets()
	{
		vector<unsigned char> bytes = outbox.GetBytesAndClear();
		Send(Packet::CreateAutoLength(bytes));
	}
}
