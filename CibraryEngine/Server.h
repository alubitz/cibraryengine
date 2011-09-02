#pragma once

#include "StdAfx.h"
#include "Disposable.h"

namespace CibraryEngine
{
	class Connection;

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

			void SendAll(Packet p);
			bool SendToClient(int id, Packet p);

			void BufferedSendAll(Packet p);
			void SendBufferedPackets();

			bool HasTerminated();

			Connection* GetConnection(int id);
	};
}
