#include "StdAfx.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

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

		SOCKET socket;

		bool disconnected;

		Imp(Server* server, SOCKET socket, unsigned int id) :
			server(server),
			id(id),
			socket(socket),
			disconnected(false)
		{
			// TODO: implement this
		}

		~Imp() { Disconnect(); }

		void Disconnect()
		{
			if (!disconnected)
            {
                //if (clientSocket.Connected)
                //    clientSocket.Close();
                disconnected = true;

                //server.GotDisconnectedFromClient(this);
            }
		}
		bool IsConnected() { return !disconnected; }

		void Send(Packet p)
		{
			// TODO: implement this
		}
	};




	/*
	 * ServerConnection methods
	 */
	ServerConnection::ServerConnection(Server* server, UINT_PTR socket, unsigned int id) : Connection(), imp(new Imp(server, socket, id)) { }

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
