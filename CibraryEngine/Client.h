#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "Connection.h"

namespace CibraryEngine
{
	using namespace std;

	class Client : public Connection
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			Client();

			unsigned int GetClientID();

			void Send(Packet p);

			void Connect(const string& server_ip_string, unsigned short port_num);
			void Disconnect();
			bool IsConnected();

			EventDispatcher Connected;

			struct ConnectedEvent : public Event
			{
				Client* client;
				ConnectedEvent(Client* client) : client(client) { }
			};
	};
}
