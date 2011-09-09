#include "StdAfx.h"

#include <boost/asio.hpp>

#include "Server.h"
#include "ServerConnection.h"

#include "Packet.h"

namespace CibraryEngine
{
	using namespace std;
	using namespace boost::asio;

	/*
	 * Server private implementation struct
	 */
	struct Server::Imp
	{
		Server* server;

		unsigned int next_client_id;
		int port_num;

		map<unsigned int, ServerConnection*> connections;

		bool started;
		bool terminated;

		ip::tcp::acceptor* socket;
		ip::tcp::socket* next_socket;

		Imp(Server* server, int port_num) :
			server(server),
			next_client_id(1),
			port_num(port_num),
			connections(),
			socket(NULL),
			next_socket(NULL)
		{
		}

		~Imp()
		{ 
			Disconnect();

			if(next_socket != NULL)
			{
				delete next_socket;
				next_socket = NULL;
			}
		}

		void BufferedSendAll(Packet p)
		{
			for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
				iter->second->BufferedSend(p);
		}

		void SendBufferedPackets()
		{
			for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
				iter->second->SendBufferedPackets();
		}

		list<unsigned int> GetClientIDs()
		{
			list<unsigned int> ids;
			for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
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

		void Start()
		{
			// TODO: synchronize this
			if(!started)
			{
				ip::tcp::endpoint endpoint(ip::tcp::v4(), port_num);
			
				socket = new ip::tcp::acceptor(io_service());
				socket->open(endpoint.protocol());
				socket->bind(endpoint);
				socket->listen();

				// TODO: trigger started listening event here

				next_socket = new ip::tcp::socket(io_service());

				MyAcceptHandler handler(this);
				socket->async_accept(*next_socket, handler);

				started = true;
			}
		}

		void Disconnect()
		{
			if (started && !terminated)
            {
                // TODO: syncrhonize this
                if (socket != NULL)
                {
                    // TODO: trigger disconnected event here

                    socket->close();

					for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
					{
						iter->second->Send(Packet::CreateFixedLength("BYE", NULL, 0));
                        iter->second->Disconnect();
                    }

					delete socket;
					socket = NULL;
                }

				terminated = true;
            }
		}

		bool IsActive() { return started && !terminated; }

		struct MyAcceptHandler
		{
			Imp* imp;

			MyAcceptHandler(Imp* imp) : imp(imp) { }

			void operator() (const boost::system::error_code& error)
			{
				unsigned int id = imp->next_client_id++;
				imp->connections[id] = new ServerConnection(imp->server, imp->next_socket, id);

				// TODO: trigger connection incoming event

				if(!imp->terminated)
				{
					imp->next_socket = new ip::tcp::socket(io_service());

					MyAcceptHandler handler(imp);
					imp->socket->async_accept(*imp->next_socket, handler);
				}
				else
					imp->next_socket = NULL;
			}
		};
	};




	/*
	 * Server methods
	 */
	Server::Server(int port_num) : imp(new Imp(this, port_num)) { }

	void Server::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void Server::Start() { imp->Start(); }
	void Server::Disconnect() { imp->Disconnect(); }
	bool Server::IsActive() { return imp->IsActive(); }

	void Server::BufferedSendAll(Packet p) { imp->BufferedSendAll(p); }
	void Server::SendBufferedPackets() { imp->SendBufferedPackets(); }

	list<unsigned int> Server::GetClientIDs() { return imp->GetClientIDs(); }
	ServerConnection* Server::GetConnection(unsigned int id) { return imp->GetConnection(id); }
}
