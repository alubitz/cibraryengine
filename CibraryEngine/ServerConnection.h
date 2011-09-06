#pragma once

#include "StdAfx.h"

#include "Connection.h"

namespace CibraryEngine
{
	class Server;

	class ServerConnection : public Connection
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			ServerConnection(Server* server, unsigned int id);

			Server* GetServer();
			unsigned int GetClientID();

			void Send(Packet p);

			void Disconnect();
			bool IsConnected();
	};
}
