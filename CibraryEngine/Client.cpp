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

		Imp(string server_ip_string, int port_num) :
			server_ip_string(server_ip_string),
			port_num(port_num),
			id(0)
		{
			// TODO: implement this
		}

		void Connect()
		{
			// TODO: implement this
		}
	};




	/*
	 * Client methods
	 */
	Client::Client(string server_ip_string, int port_num) : imp(new Imp(server_ip_string, port_num)) { }

	void Client::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	unsigned int Client::GetClientID()
	{
		return imp->id;
	}

	void Client::Send(Packet p)
	{
		// TODO: implement this
	}

	void Client::Connect() { imp->Connect(); }

	void Client::Disconnect()
	{
		// TODO: implement this
	}

	bool Client::IsConnected()
	{
		// TODO: implement this
		return false;
	}
}
