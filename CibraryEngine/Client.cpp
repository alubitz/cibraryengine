#include "StdAfx.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

#include "Client.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * Client private implementation struct
	 */
	struct Client::Imp
	{
		string server_ip_string;
		int port_num;

		unsigned int id;

		bool started;
		bool connected;
		bool terminated;

		SOCKET socket;

		Imp(string server_ip_string, int port_num) :
			server_ip_string(server_ip_string),
			port_num(port_num),
			id(0),
			started(false),
			connected(false),
			terminated(false),
			socket(INVALID_SOCKET)
		{
			 //socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
		}

		~Imp() { Disconnect(); }

		void Connect()
		{
			if(!started)
			{
				started = true;

				//socket.BeginConnect(endpoint, UponConnection, socket);
			}
		}

		void Disconnect()
		{
			if (connected && !terminated)
            {
                //lock (this)
                //{
                    terminated = true;
                    if (socket != NULL)
                    {
                        //if (Disconnected != null) { Disconnected(this); }
                        //socket.Close();
                    }
                //}
            }
		}

		bool IsConnected() { return connected && !terminated; }

		void Send(Packet p)
		{
			if (connected && !terminated)
            {
				// TODO: implement this
            }
		}
	};




	/*
	 * Client methods
	 */
	Client::Client(string server_ip_string, int port_num) : Connection(), imp(new Imp(server_ip_string, port_num)) { }

	void Client::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void Client::Connect() { imp->Connect(); }
	void Client::Disconnect() { imp->Disconnect(); }
	bool Client::IsConnected() { return imp->IsConnected(); }

	void Client::Send(Packet p) { imp->Send(p); }

	unsigned int Client::GetClientID() { return imp->id; }
}
