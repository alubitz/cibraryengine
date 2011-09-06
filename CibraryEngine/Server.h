#pragma once

#include "StdAfx.h"
#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	class ServerConnection;

	struct Packet;

	class Server
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			Server(int port_num);

			void Start();
			void Disconnect();

			void BufferedSendAll(Packet p);
			void SendBufferedPackets();

			bool IsActive();

			list<unsigned int> GetClientIDs();
			ServerConnection* GetConnection(unsigned int id);
	};
}
