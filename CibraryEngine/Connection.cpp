#include "StdAfx.h"

#include "Connection.h"
#include "Packet.h"

#include "Inbox.h"
#include "Outbox.h"

namespace CibraryEngine
{
	using namespace boost::asio;




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

	void Connection::SendBufferedPackets() { Send(Packet::CreateAutoLength(outbox.GetBytesAndClear())); }
}
