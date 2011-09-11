#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Inbox.h"
#include "Outbox.h"

#include "Events.h"

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

			EventDispatcher Disconnected;
			EventDispatcher BytesReceived;
			EventDispatcher PacketReceived;
			EventDispatcher BytesSent;
			EventDispatcher ConnectionError;

			struct DisconnectedEvent : public Event
			{
				Connection* connection;
				DisconnectedEvent(Connection* connection) : connection(connection) { }
			};

			struct BytesReceivedEvent : public Event
			{
				Connection* connection;
				vector<unsigned char> bytes;
				BytesReceivedEvent(Connection* connection, vector<unsigned char> bytes) : connection(connection), bytes(bytes) { }
			};

			struct PacketReceivedEvent : public Event
			{
				Connection* connection;
				ReceivedPacket packet;
				PacketReceivedEvent(Connection* connection, ReceivedPacket packet) : connection(connection), packet(packet) { }
			};

			struct BytesSentEvent : public Event
			{
				Connection* connection;
				vector<unsigned char> bytes;
				BytesSentEvent(Connection* connection, vector<unsigned char> bytes) : connection(connection), bytes(bytes) { }
			};

			struct ConnectionErrorEvent : public Event
			{
				Connection* connection;
				boost::system::error_code error;
				ConnectionErrorEvent(Connection* connection, boost::system::error_code error) : connection(connection), error(error) { }
			};
	};
}
