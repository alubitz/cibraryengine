#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Events.h"

#include "ReceivedPacket.h"

namespace CibraryEngine
{
	using namespace std;

	class ServerConnection;

	struct Packet;

	class Server : public Disposable
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			Server();

			void Start(unsigned short port_num);
			void Disconnect();
			void DisconnectClient(unsigned int id);

			void BufferedSendAll(Packet p);
			void SendBufferedPackets();

			bool IsActive();

			list<unsigned int> GetClientIDs();
			ServerConnection* GetConnection(unsigned int id);

			EventDispatcher BeganListening;
			EventDispatcher IncomingConnection;
			EventDispatcher ServerDisconnected;
			EventDispatcher ClientDisconnected;
			EventDispatcher BytesReceived;
			EventDispatcher PacketReceived;
			EventDispatcher BytesSent;
			EventDispatcher ConnectionError;
			EventDispatcher ServerError;

			struct BeganListeningEvent : public Event
			{
				Server* server;
				BeganListeningEvent(Server* server) : server(server) { }
			};

			struct IncomingConnectionEvent : public Event
			{
				Server* server;
				ServerConnection* connection;
				IncomingConnectionEvent(Server* server, ServerConnection* connection) : server(server), connection(connection) { }
			};

			struct DisconnectedEvent : public Event
			{
				Server* server;
				DisconnectedEvent(Server* server) : server(server) { }
			};

			struct ClientDisconnectedEvent : public Event
			{
				Server* server;
				ServerConnection* connection;
				ClientDisconnectedEvent(Server* server, ServerConnection* connection) : server(server), connection(connection) { }
			};

			struct BytesReceivedEvent : public Event
			{
				ServerConnection* connection;
				vector<unsigned char> bytes;
				BytesReceivedEvent(ServerConnection* connection, vector<unsigned char> bytes) : connection(connection), bytes(bytes) { }
			};

			struct PacketReceivedEvent : public Event
			{
				ServerConnection* connection;
				ReceivedPacket packet;
				PacketReceivedEvent(ServerConnection* connection, ReceivedPacket packet) : connection(connection), packet(packet) { }
			};

			struct BytesSentEvent : public Event
			{
				ServerConnection* connection;
				vector<unsigned char> bytes;
				BytesSentEvent(ServerConnection* connection, vector<unsigned char> bytes) : connection(connection), bytes(bytes) { }
			};

			struct ServerErrorEvent : public Event
			{
				Server* server;
				boost::system::error_code error;
				ServerErrorEvent(Server* server, boost::system::error_code error) : server(server), error(error) { }
			};
	};
}
