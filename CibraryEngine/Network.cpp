#include "StdAfx.h"

#include "Network.h"

// includes for Network::DoTestProgram
#include "Server.h"
#include "ServerConnection.h"
#include "Client.h"
#include "DebugLog.h"

namespace CibraryEngine
{
	using namespace boost::asio;




	/*
	 * Network methods
	 */
	io_service* global_io_service = NULL;
	io_service& Network::GetIOService()
	{
		if(global_io_service == NULL)
			global_io_service = new io_service();

		return *global_io_service;
	}

	void DoAsyncStuff();

	boost::thread* async_thread = NULL;
	bool async_go = false;
	void Network::StartAsyncSystem()
	{
		if(async_thread == NULL)
		{
			async_go = true;
			async_thread = new boost::thread(DoAsyncStuff);
		}
	}

	void Network::StopAsyncSystem()
	{
		if(async_go)
			async_go = false;
	}

	void DoAsyncStuff()
	{
		while(async_go)
			Network::GetIOService().run_one();
	}




	/*
	 * Test program for network stuff
	 */
	void Network::DoTestProgram()
	{
		Network::StartAsyncSystem();

		Server server;
		bool loop = 1;

		struct IncomingConnectionHandler : public EventHandler
		{
			void HandleEvent(Event* evt)
			{
				Debug("Server event: incoming connection\n");
			}
		} incoming_connection;

		struct BeganListeningHandler : public EventHandler
		{
			void HandleEvent(Event* evt)
			{
				Debug("Server event: began listening\n");
			}
		} began_listening;

		struct ServerErrorHandler : public EventHandler
		{
			bool* loop;
			void HandleEvent(Event* evt)
			{
				stringstream ss;
				ss << "server error: " << ((Server::ServerErrorEvent*)evt)->error << endl;
				Debug(ss.str());

				*loop = false;
			}
		} server_error_handler;
		server_error_handler.loop = &loop;

		struct ConnectionErrorHandler : public EventHandler
		{
			bool* loop;
			void HandleEvent(Event* evt)
			{
				stringstream ss;
				ss << "connection error: " << ((Connection::ConnectionErrorEvent*)evt)->error << endl;
				Debug(ss.str());

				*loop = false;
			}
		} connection_error_handler;
		connection_error_handler.loop = &loop;

		struct ServerReceivedPacketHandler : public EventHandler
		{
			bool* loop;
			void HandleEvent(Event* evt)
			{
				Server::PacketReceivedEvent* pre = (Server::PacketReceivedEvent*)evt;
				ReceivedPacket packet = pre->packet;

				string type;
				string data;
				packet.packet.DecodePacket(type, data);

				stringstream ss;
				ss << "server received packet; type is \"" << type << "\"; packet body contains " << data.length() << " bytes of data" << endl;
				Debug(ss.str());

				*loop = false;
			}
		} server_received_packet_handler;
		server_received_packet_handler.loop = &loop;


		struct ServerBytesReceivedHandler : public EventHandler
		{
			void HandleEvent(Event* evt)
			{
				Server::BytesReceivedEvent* bre = (Server::BytesReceivedEvent*)evt;

				stringstream ss;
				ss << "server received " << bre->bytes.size() << " bytes from client" << endl;
				Debug(ss.str());
			}
		} server_bytes_received_handler;

		struct ClientErrorHandler : public EventHandler
		{
			bool* loop;
			void HandleEvent(Event* evt)
			{
				stringstream ss;
				ss << "client error: " << ((Connection::ConnectionErrorEvent*)evt)->error << endl;
				Debug(ss.str());

				*loop = false;
			}
		} client_error_handler;
		client_error_handler.loop = &loop;

		struct ClientBytesSentHandler : public EventHandler
		{
			void HandleEvent(Event* evt)
			{
				Connection::BytesSentEvent* bse = (Connection::BytesSentEvent*)evt;

				stringstream ss;
				ss << "client sent " << bse->bytes.size() << " bytes" << endl;
				Debug(ss.str());
			}
		} client_bytes_sent_handler;

		server.IncomingConnection += &incoming_connection;
		server.BeganListening += &began_listening;
		server.ServerError += &server_error_handler;
		server.ConnectionError += &connection_error_handler;
		server.PacketReceived += &server_received_packet_handler;
		server.BytesReceived += &server_bytes_received_handler;

		server.Start(7777);

		while(!server.IsActive()) { Sleep(1); }

		Client client;
		client.ConnectionError += &client_error_handler;
		client.BytesSent += &client_bytes_sent_handler;

		client.Connect("127.0.0.1", 7777);

		while(!client.IsConnected()) { Sleep(1); }

		client.Send(Packet::CreateNamedAutoLength("HELLO", string()));

		while(loop) { Sleep(1); }

		Debug("loop = false, exiting\n");

		client.Dispose();
		server.Dispose();
	}
}

