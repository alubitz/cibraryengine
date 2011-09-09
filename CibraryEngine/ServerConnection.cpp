#include "StdAfx.h"

#include <boost/asio.hpp>

#include "ServerConnection.h"
#include "Server.h"

namespace CibraryEngine
{
	using namespace boost::asio;

	/*
	 * ServerConnection private implementation struct
	 */
	struct ServerConnection::Imp
	{
		Server* server;
		unsigned int id;

		Inbox* inbox;

		ip::tcp::socket* socket;

		bool disconnected;

		Imp(Server* server, ip::tcp::socket* socket, unsigned int id, Inbox* inbox) :
			server(server),
			id(id),
			socket(socket),
			disconnected(false)
		{
			MyReceiveHandler handler(this);
			socket->async_receive(buffer(handler.buf), handler);
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

                // TODO: trigger disconnected from client event
            }
		}
		bool IsConnected() { return !disconnected; }

		void Send(Packet p)
		{
			// TODO: synchronize this
			if(!disconnected)
			{
				vector<unsigned char> bytes = p.GetBytes();

				socket->async_send(buffer(&bytes[0], bytes.size() * sizeof(unsigned char)), MySendHandler(this));
			}
		}

		struct MyReceiveHandler
		{
			Imp* imp;
			mutable_buffer buf;

			MyReceiveHandler(Imp* imp) : imp(imp), buf() { }


			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				if(!error)
				{
					unsigned char* my_byte_array = buffer_cast<unsigned char*>(buf);
					
					// TODO: trigger bytes received event

					list<ReceivedPacket> received = imp->inbox->Receive(my_byte_array, bytes_transferred);
					for(list<ReceivedPacket>::iterator iter = received.begin(); iter != received.end(); iter++)
					{
						// TODO: trigger packet received event
					}
					imp->inbox->ClearPackets();

					if(!imp->disconnected)
					{
						MyReceiveHandler handler(imp);
						imp->socket->async_receive(buffer(handler.buf), handler);
					}
				}
			}
		};

		struct MySendHandler
		{
			Imp* imp;
			MySendHandler(Imp* imp) : imp(imp) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred) { /* TODO: trigger a packet sent event */ }
		};
	};




	/*
	 * ServerConnection methods
	 */
	ServerConnection::ServerConnection(Server* server, ip::tcp::socket* socket, unsigned int id) :
		Connection(),
		imp(new Imp(server, socket, id, &inbox))
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
