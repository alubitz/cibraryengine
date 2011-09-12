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
		struct ImpPtr
		{
			Imp* imp;
			bool receive;
			bool send;

			boost::mutex mutex;

			ImpPtr(Imp* imp) : imp(imp), receive(false), send(false), mutex() { }

			bool CanDelete() { return imp == NULL && !receive && !send; }
		};

		ImpPtr* self;

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
			disconnected(false)
		{
		}

		~Imp()
		{ 
			{
				boost::mutex::scoped_lock lock(self->mutex);				// synchronize the following...

				self->imp = NULL;
				if(self->CanDelete())
					delete self;
			}

			Disconnect(); 
		}

		void Disconnect()
		{
			if (!disconnected)
            {
				boost::mutex::scoped_lock lock(self->mutex);				// synchronize the following...

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
			boost::mutex::scoped_lock lock(self->mutex);				// synchronize the following...

			if(!disconnected)
			{
				string bytes = p.GetBytes();
				for(unsigned int i = 0; i < bytes.length(); i++)
					my_send_handler.bytes[i] = bytes[i];

				self->send = true;
				socket->async_send(buffer(&my_send_handler.bytes[0], bytes.length() * sizeof(unsigned char)), my_send_handler);
			}
		}

		struct MyReceiveHandler
		{
			ImpPtr* ptr;
			vector<unsigned char> bytes;
			mutable_buffer buf;

			MyReceiveHandler() : ptr(NULL), bytes(1024), buf(&bytes[0], 1024 * sizeof(unsigned char)) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				boost::mutex::scoped_lock lock(ptr->mutex);				// synchronize the following...

				ptr->receive = false;
				Imp* imp = ptr->imp;
				if(imp != NULL)
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
				if(ptr->CanDelete())
					delete ptr;
			}
		} my_receive_handler;

		struct MySendHandler
		{
			ImpPtr* ptr;
			vector<unsigned char> bytes;

			MySendHandler() : ptr(NULL), bytes(1024) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				boost::mutex::scoped_lock lock(ptr->mutex);				// synchronize the following...

				ptr->send = false;

				Imp* imp = ptr->imp;
				if(imp != NULL)
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
				if(ptr->CanDelete())
					delete ptr;
			}
		} my_send_handler;
	};

	void ServerConnection::Imp::AsyncReceive()
	{
		self->receive = true;
		socket->async_receive(buffer(my_receive_handler.buf), my_receive_handler);
	}




	/*
	 * ServerConnection methods
	 */
	ServerConnection::ServerConnection(Server* server, ip::tcp::socket* socket, unsigned int id) :
		Connection()
	{
		imp = new Imp(server, this, socket, id, &inbox);
		imp->my_send_handler.ptr = imp->my_receive_handler.ptr = imp->self = new Imp::ImpPtr(imp);
		imp->AsyncReceive();
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
