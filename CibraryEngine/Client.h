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

			Client(string server_ip_string, int port_num);

			unsigned int GetClientID();

			void Send(Packet p);

			void Connect();
			void Disconnect();
			bool IsConnected();
	};
}
