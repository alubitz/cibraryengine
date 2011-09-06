#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Inbox.h"
#include "Outbox.h"

namespace CibraryEngine
{
	/**
	 * A client, or a server's connection to a client; basically just a wrapper for sockets
	 */
	class Connection : public Disposable
	{
		protected:

			virtual void InnerDispose();

		public:

			Inbox inbox;
			Outbox outbox;

			Connection();

			// no Connect method because that isn't applicable for a server

			virtual void Send(Packet p) = 0;

			virtual void BufferedSend(Packet p);
			virtual void SendBufferedPackets();

			virtual void Disconnect() = 0;
			virtual bool IsConnected() = 0;
	};
}
