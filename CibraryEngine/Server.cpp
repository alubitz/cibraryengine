#include "StdAfx.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

#include "Server.h"
#include "ServerConnection.h"

#include "Packet.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * Server private implementation struct
	 */
	struct Server::Imp
	{
		unsigned int next_client_id;
		int port_num;

		map<unsigned int, ServerConnection*> connections;

		bool started;
		bool terminated;

		SOCKET socket;

		Imp(int port_num) :
			next_client_id(1),
			port_num(port_num),
			connections(),
			socket(INVALID_SOCKET)
		{
			// serverSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            // localEndPoint = new IPEndPoint(IPAddress.Any, port_num);
		}

		~Imp() { Disconnect(); }

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
			// TODO: implement this
		}

		void Disconnect()
		{
			if (started && !terminated)
            {
                //lock (this)
                //{
                    terminated = true;
                    if (socket != INVALID_SOCKET)
                    {
                        //if (Disconnected != null) { Disconnected(this); }

                        //serverSocket.Close();

                        //lock (connections)
                        //{
							for(map<unsigned int, ServerConnection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
							{
								iter->second->Send(Packet::CreateFixedLength("BYE", NULL, 0));
                                iter->second->Disconnect();
                            }
                        //}
                    }
                //}
            }
		}

		bool IsActive() { return started && !terminated; }
	};




	/*
	 * Server methods
	 */
	Server::Server(int port_num) : imp(new Imp(port_num)) { }

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
