#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "Connection.h"

namespace CibraryEngine
{
	using namespace std;

	class Client : public Connection
	{
		protected:

			void InnerDispose();

		public:

			Client(string server_ip_string, int port_num);

			void Connect();
			void Disconnect();

			void BufferedSend(Packet p);
			void SendBufferedPackets();
	};
}
