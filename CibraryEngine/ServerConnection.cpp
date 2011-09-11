#include "StdAfx.h"

#include "ServerConnection.h"
#include "Server.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	using namespace boost::asio;

	/*
	 * ServerConnection private implementation struct
	 */
	struct ServerConnection::Imp
	{
		Server* server;
		ServerConnection* connection;
		unsigned int id;

		ip::tcp::socket* socket;

		bool disconnected;

		void AsyncReceive();

		Imp(Server* server, ServerConnection* connection, ip::tcp::socket* socket, unsigned int id, Inbox* inbox) :
			server(server),
			connection(connection),
			id(id),
			socket(socket),
			disconnected(false),
			my_receive_handler(this),
			my_send_handler(this)
		{
			AsyncReceive();
		}

		~Imp() { Disconnect(); }

		void Disconnect()
		{
			// TODO: synchronize this
			if (!disconnected)
            {
				if(socket != NULL)
				{
					if(socket->is_open())
						socket->close();

					delete socket;
					socket = NULL;
				}

                disconnected = true;

				Connection::DisconnectedEvent evt(connection);
				connection->Disconnected(&evt);

				server->DisconnectClient(connection->GetClientID());
            }
		}
		bool IsConnected() { return !disconnected; }

		void Send(Packet p)
		{
			// TODO: synchronize this
			if(!disconnected)
			{
				vector<unsigned char> bytes = p.GetBytes();
				for(unsigned int i = 0; i < bytes.size(); i++)
					my_send_handler.bytes[i] = bytes[i];

				socket->async_send(buffer(&my_send_handler.bytes[0], bytes.size() * sizeof(unsigned char)), my_send_handler);
			}
		}

		struct MyReceiveHandler
		{
			Imp* imp;
			vector<unsigned char> bytes;
			mutable_buffer buf;

			MyReceiveHandler(Imp* imp) : imp(imp), bytes(1024), buf(&bytes[0], 1024 * sizeof(unsigned char)) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				if(!error)
				{		
					unsigned char* my_byte_array = buffer_cast<unsigned char*>(buf);

					vector<unsigned char> packet_bytes(bytes_transferred);
					for(unsigned int i = 0; i < bytes_transferred; i++)
						packet_bytes[i] = my_byte_array[i];

					Connection::BytesReceivedEvent evt(imp->connection, packet_bytes);
					imp->connection->BytesReceived(&evt);

					list<ReceivedPacket> received = imp->connection->inbox.Receive(&packet_bytes[0], bytes_transferred);
					for(list<ReceivedPacket>::iterator iter = received.begin(); iter != received.end(); iter++)
					{
						Connection::PacketReceivedEvent evt(imp->connection, *iter);
						imp->connection->PacketReceived(&evt);
					}
					imp->connection->inbox.ClearPackets();

					if(!imp->disconnected)
						imp->AsyncReceive();
				}
				else
				{
					Connection::ConnectionErrorEvent evt(imp->connection, error);
					imp->connection->ConnectionError(&evt);
				}
			}
		} my_receive_handler;

		struct MySendHandler
		{
			Imp* imp;
			vector<unsigned char> bytes;
			MySendHandler(Imp* imp) : imp(imp), bytes(1024) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				if(!error)
				{
					vector<unsigned char> sent_bytes(bytes_transferred);
					for(unsigned int i = 0; i < bytes_transferred; i++)
						sent_bytes[i] = bytes[i];

					Connection::BytesSentEvent evt(imp->connection, sent_bytes);
					imp->connection->BytesSent(&evt);
				}
				else
				{
					Server::ServerErrorEvent evt(imp->server, error);
					imp->server->ConnectionError(&evt);
				}
			}
		} my_send_handler;
	};

	void ServerConnection::Imp::AsyncReceive()
	{
		socket->async_receive(buffer(my_receive_handler.buf), my_receive_handler);
	}




	/*
	 * ServerConnection methods
	 */
	ServerConnection::ServerConnection(Server* server, ip::tcp::socket* socket, unsigned int id) :
		Connection(),
		imp(new Imp(server, this, socket, id, &inbox))
	{
	}

	void ServerConnection::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	Server* ServerConnection::GetServer() { return imp->server; }
	unsigned int ServerConnection::GetClientID() { return imp->id; }

	void ServerConnection::Send(Packet p) { imp->Send(p); }
	void ServerConnection::Disconnect() { imp->Disconnect(); }
	bool ServerConnection::IsConnected() { return imp->IsConnected(); }
}
