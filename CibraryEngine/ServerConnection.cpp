#include "StdAfx.h"

#include "ServerConnection.h"
#include "Server.h"

namespace CibraryEngine
{
	/*
	 * ServerConnection private implementation struct
	 */
	struct ServerConnection::Imp
	{
		Server* server;
		unsigned int id;

		Imp(Server* server, unsigned int id) :
			server(server),
			id(id)
		{
			// TODO: implement this
		}
	};




	/*
	 * ServerConnection methods
	 */
	ServerConnection::ServerConnection(Server* server, unsigned int id) : imp(new Imp(server, id)) { }

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

	void ServerConnection::Send(Packet p)
	{
		// TODO: implement this
	}
	void ServerConnection::Disconnect()
	{
		// TODO: implement this
	}
	bool ServerConnection::IsConnected()
	{
		// TODO: implement this
		return false;
	}
}
