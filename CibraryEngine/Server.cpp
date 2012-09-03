#include "StdAfx.h"
#include "DebugLog.h"

#include "Server.h"
#include "ServerConnection.h"
#include "Network.h"

#include "Packet.h"

namespace CibraryEngine
{
	using namespace std;
	using namespace boost::asio;

	/*
	 * Handlers for passing client (ServerConnection) events to the server event dispatchers
	 */
	void RemoveDefaultHandlers(ServerConnection* connection);

	struct ClientDisconnectedHandler : public EventHandler
	{ 
		void HandleEvent(Event* evt)
		{
			Connection::DisconnectedEvent* cde = (Connection::DisconnectedEvent*)evt; 
			ServerConnection* connection = (ServerConnection*)cde->connection;
			Server* server = connection->GetServer();
			Server::ClientDisconnectedEvent server_evt(server, connection);
			server->ClientDisconnected(&server_evt);
		} 
	} client_disconnected_handler;

	struct BytesReceivedHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Connection::BytesReceivedEvent* bre = (Connection::BytesReceivedEvent*)evt;
			ServerConnection* connection = (ServerConnection*)bre->connection;
			Server* server = connection->GetServer();
			Server::BytesReceivedEvent server_evt(connection, bre->bytes);
			server->BytesReceived(&server_evt);
		}
	} bytes_received_handler;

	struct PacketReceivedHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Connection::PacketReceivedEvent* pre = (Connection::PacketReceivedEvent*)evt;
			ServerConnection* connection = (ServerConnection*)pre->connection;
			Server* server = connection->GetServer();
			Server::PacketReceivedEvent server_evt(connection, pre->packet);
			server->PacketReceived(&server_evt);
		}
	} packet_received_handler;

	struct BytesSentHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Connection::BytesSentEvent* bse = (Connection::BytesSentEvent*)evt;
			ServerConnection* connection = (ServerConnection*)bse->connection;
			Server* server = connection->GetServer();
			Server::BytesSentEvent server_evt(connection, bse->bytes);
			server->BytesSent(&server_evt);
		}
	} bytes_sent_handler;

	struct ConnectionErrorHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Connection::ConnectionErrorEvent* cee = (Connection::ConnectionErrorEvent*)evt;
			ServerConnection* connection = (ServerConnection*)cee->connection;
			Server* server = connection->GetServer();
			server->ConnectionError(cee);
		}
	} connection_error_handler;

	void RemoveDefaultHandlers(ServerConnection* connection)
	{
		connection->Disconnected -= &client_disconnected_handler;
		connection->BytesReceived -= &bytes_received_handler;
		connection->PacketReceived -= &packet_received_handler;
		connection->BytesSent -= &bytes_sent_handler;
		connection->ConnectionError -= &connection_error_handler;
	}




	/*
	 * Server private implementation struct
	 */
	struct Server::Imp
	{
		struct ImpPtr
		{
			Imp* imp;
			bool accept;

			boost::mutex mutex;

			ImpPtr(Imp* imp) : imp(imp), accept(false), mutex() { }

			bool CanDelete() { return imp == NULL && !accept; }
		};

		Server* server;
		ImpPtr* self;

		unsigned int next_client_id;
		unsigned short port_num;

		map<unsigned int, ServerConnection*> connections;

		bool started;
		bool terminated;

		ip::tcp::acceptor* socket;
		ip::tcp::socket* next_socket;

		Imp(Server* server) :
			server(server),
			next_client_id(1),
			port_num(0),
			connections(),
			started(false),
			terminated(false),
			socket(NULL),
			next_socket(NULL)
		{
		}

		~Imp()
		{
			// curly braces to limit scope of lock
			{
				boost::mutex::scoped_lock lock(self->mutex);				// synchronize the following...

				self->imp = NULL;
				if(self->CanDelete())
				{
					delete self;
					self = NULL;
				}
			}

			Disconnect();

			if(next_socket != NULL)
			{
				delete next_socket;
				next_socket = NULL;
			}
		}

		void BufferedSendAll(Packet p)
		{
			for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); ++iter)
				iter->second->BufferedSend(p);
		}

		void SendBufferedPackets()
		{
			boost::mutex::scoped_lock lock(self->mutex);				// synchronize the following...

			for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); ++iter)
				iter->second->SendBufferedPackets();
		}

		list<unsigned int> GetClientIDs()
		{
			list<unsigned int> ids;
			for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); ++iter)
				ids.push_back(iter->first);
			return ids;
		}

		ServerConnection* GetConnection(unsigned int id)
		{
			map<unsigned int, ServerConnection*>::iterator found = connections.find(id);
			if(found != connections.end())
				return found->second;
			else
				return NULL;
		}

		void AsyncAccept();

		void Start(unsigned short port_num)
		{
			boost::mutex::scoped_lock lock(self->mutex);				// synchronize the following...
			
			if(!started)
			{
				ip::tcp::endpoint endpoint(ip::tcp::v4(), port_num);
			
				socket = new ip::tcp::acceptor(Network::GetIOService());
				socket->open(endpoint.protocol());
				socket->bind(endpoint);
				socket->listen();

				Server::BeganListeningEvent evt(server);
				server->BeganListening(&evt);

				AsyncAccept();

				this->port_num = port_num;
				started = true;
			}
		}

		void Disconnect()
		{
			boost::mutex::scoped_lock lock(self->mutex);				// synchronize the following...

			if(started && !terminated)
            {
                if(socket != NULL)
                {
					Server::DisconnectedEvent evt(server);
					server->ServerDisconnected(&evt);

                    socket->close();

					for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); ++iter)
					{
						iter->second->Send(Packet::CreateNamedAutoLength("BYE", string()));

						RemoveDefaultHandlers(iter->second);
                        iter->second->Disconnect();
						iter->second->Dispose();

						delete iter->second;
                    }
					connections.clear();

					delete socket;
					socket = NULL;
                }

				terminated = true;
            }
		}

		void DisconnectClient(unsigned int id)
		{
			ServerConnection* connection = GetConnection(id);
			RemoveDefaultHandlers(connection);

			connection->Disconnect();
		}

		bool IsActive() { return started && !terminated; }

		struct MyAcceptHandler
		{
			ImpPtr* ptr;

			MyAcceptHandler() : ptr(NULL) { }

			void operator() (const boost::system::error_code& error)
			{
				boost::mutex::scoped_lock lock(ptr->mutex);				// synchronize the following...

				ptr->accept = false;
				Imp* imp = ptr->imp;

				if(imp != NULL)
				{
					if(!error)
					{
						unsigned int id = imp->next_client_id++;
						
						ServerConnection* connection = new ServerConnection(imp->server, imp->next_socket, id);
						imp->connections[id] = connection;

						Server::IncomingConnectionEvent evt(imp->server, connection);
						imp->server->IncomingConnection(&evt);
						
						connection->Disconnected += &client_disconnected_handler;
						connection->BytesReceived += &bytes_received_handler;
						connection->PacketReceived += &packet_received_handler;
						connection->BytesSent += &bytes_sent_handler;
						connection->ConnectionError += &connection_error_handler;

						if(!imp->terminated)
							imp->AsyncAccept();
						else
							imp->next_socket = NULL;
					}
					else
					{
						ServerErrorEvent evt(imp->server, error);
						imp->server->ServerError(&evt);
					}
				}

				if(ptr->CanDelete())
					delete ptr;
			}
		} my_accept_handler;
	};

	void Server::Imp::AsyncAccept()
	{
		self->accept = true;
		next_socket = new ip::tcp::socket(Network::GetIOService());
		socket->async_accept(*next_socket, my_accept_handler);
	}


	/*
	 * Server methods
	 */
	Server::Server()
	{
		imp = new Imp(this);
		imp->my_accept_handler.ptr = imp->self = new Imp::ImpPtr(imp);
	}

	void Server::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void Server::Start(unsigned short port_num) { imp->Start(port_num); }
	void Server::Disconnect() { imp->Disconnect(); }
	void Server::DisconnectClient(unsigned int id) { imp->DisconnectClient(id); }
	bool Server::IsActive() { return imp->IsActive(); }

	void Server::BufferedSendAll(Packet p) { imp->BufferedSendAll(p); }
	void Server::SendBufferedPackets() { imp->SendBufferedPackets(); }

	list<unsigned int> Server::GetClientIDs() { return imp->GetClientIDs(); }
	ServerConnection* Server::GetConnection(unsigned int id) { return imp->GetConnection(id); }
}
