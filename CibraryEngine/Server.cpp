#include "StdAfx.h"

#include "Server.h"
#include "ServerConnection.h"

#include "Packet.h"

#include <winsock.h>

namespace CibraryEngine
{
	using namespace std;

	/*
	 * Server private implementation struct
	 */
	struct Server::Imp
	{
		int port_num;
		map<unsigned int, Connection*> connections;

		Imp(int port_num) :
			port_num(port_num),
			connections()
		{
		}

		~Imp() { }

		bool SendToClient(unsigned int id, Packet p)
		{
			map<unsigned int, Connection*>::iterator found = connections.find(id);
			if(found != connections.end())
			{
				found->second->Send(p);
				return true;
			}
			else
				return false;
		}

		void BufferedSendAll(Packet p)
		{
			for(map<unsigned int, Connection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
				iter->second->BufferedSend(p);
		}

		void SendBufferedPackets()
		{
			for(map<unsigned int, Connection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
				iter->second->SendBufferedPackets();
		}
	};




	/*
	 * Server methods
	 */
	Server::Server(int port_num) :
		imp(new Imp(port_num))
	{
		// TODO: implement this
	}

	void Server::InnerDispose()
	{
		Disconnect();

		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void Server::Start()
	{
		// TODO: implement this
	}
	void Server::Disconnect()
	{
		// TODO: implement this
	}

	void Server::SendAll(Packet p)
	{
		for(map<unsigned int, Connection*>::iterator iter = imp->connections.begin(); iter != imp->connections.end(); iter++)
			SendToClient(iter->first, p);
	}
	bool Server::SendToClient(int id, Packet p) { return imp->SendToClient(id, p); }

	void Server::BufferedSendAll(Packet p) { imp->BufferedSendAll(p); }
	void Server::SendBufferedPackets() { imp->SendBufferedPackets(); }

	bool Server::IsActive()
	{
		// TODO: implement this
		return false; 
	}

	ServerConnection* Server::GetConnection(int id)
	{
		// TODO: implement this
		return NULL;
	}
}
