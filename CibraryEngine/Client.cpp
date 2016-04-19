#include "StdAfx.h"

#include "Client.h"
#include "Network.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	using namespace boost::asio;

	/*
	 * Client private implementation struct
	 */
	struct Client::Imp
	{
		struct ImpPtr
		{
			Imp* imp;
			bool send;
			bool receive;
			bool connect;

			mutex mutex;

			ImpPtr(Imp* imp) : imp(imp), send(false), receive(false), connect(false), mutex() { }

			bool CanDelete() { return imp == NULL && !send && !receive && !connect; }
		};

		Client* client;
		ImpPtr* self;

		string server_ip_string;
		unsigned short port_num;

		unsigned int id;

		bool started;
		bool connected;
		bool terminated;

		Inbox* inbox;

		boost::asio::ip::tcp::socket* socket;

		Imp(Client* client, Inbox* inbox) :
			client(client),
			server_ip_string(),
			port_num(0),
			id(0),
			started(false),
			connected(false),
			terminated(false),
			inbox(inbox),
			socket(NULL)
		{
		}

		~Imp()
		{
			{
				unique_lock<mutex> lock(self->mutex);				// synchronize this block of code...

				self->imp = NULL;
				if(self->CanDelete())
				{
					delete self;
					self = NULL;
				}
			}

			Disconnect();
		}

		void Connect(const string& server_ip_string, unsigned short port_num)
		{
			unique_lock<mutex> lock(self->mutex);				// synchronize the following...

			if(!started)
			{
				started = true;

				socket = new ip::tcp::socket(Network::GetIOService());

				ip::address address = ip::address_v4::from_string(server_ip_string.c_str());
				ip::tcp::endpoint endpoint(address, port_num);
				
				this->server_ip_string = server_ip_string;
				this->port_num = port_num;

				self->connect = true;
				socket->async_connect(endpoint, my_connect_handler);
			}
		}

		void Disconnect()
		{
			unique_lock<mutex> lock(self->mutex);				// synchronize the following...

			if(connected && !terminated)
			{
				terminated = true;
				if(socket != NULL)
				{
					Connection::DisconnectedEvent evt(client);
					client->Disconnected(&evt);

					socket->close();

					delete socket;
					socket = NULL;
				}
			}
		}

		bool IsConnected() { return connected && !terminated; }

		void Send(Packet p)
		{
			unique_lock<mutex> lock(self->mutex);				// synchronize the following...

			if(connected && !terminated)
			{
				string bytes = p.GetBytes();
				for(unsigned int i = 0; i < bytes.length(); ++i)
					my_send_handler.bytes[i] = bytes[i];

				self->send = true;
				socket->async_send(buffer(&my_send_handler.bytes[0], bytes.length() * sizeof(unsigned char)), my_send_handler);
			}
		}

		void AsyncReceive();

		struct MyConnectHandler
		{
			ImpPtr* ptr;

			MyConnectHandler() : ptr(NULL) { }

			void operator ()(const boost::system::error_code& error) 
			{
				unique_lock<mutex> lock(ptr->mutex);				// synchronize the following...

				ptr->connect = false;
				Imp* imp = ptr->imp;

				if(imp != NULL)
				{
					if(!error)
					{
						imp->connected = true;

						Client::ConnectedEvent evt(imp->client);
						imp->client->Connected(&evt);

						imp->AsyncReceive();
					}
					else
					{
						Connection::ConnectionErrorEvent evt(imp->client, error);
						imp->client->ConnectionError(&evt);
					}
				}

				if(ptr->CanDelete())
					delete ptr;
			}
		} my_connect_handler;

		struct MyReceiveHandler
		{
			ImpPtr* ptr;
			vector<unsigned char> bytes;
			mutable_buffer buf;

			MyReceiveHandler() : ptr(NULL), bytes(1024), buf(&bytes[0], 1024 * sizeof(unsigned char)) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				unique_lock<mutex> lock(ptr->mutex);				// synchronize the following...

				ptr->receive = false;
				Imp* imp = ptr->imp;

				if(imp)
				{
					if(!error)
					{
						unsigned char* my_byte_array = buffer_cast<unsigned char*>(buf);

						vector<unsigned char> packet_bytes(bytes_transferred);
						for(unsigned int i = 0; i < bytes_transferred; ++i)
							packet_bytes[i] = my_byte_array[i];

						Connection::BytesReceivedEvent evt(imp->client, packet_bytes);
						imp->client->BytesReceived(&evt);

						list<ReceivedPacket> received = imp->inbox->Receive(my_byte_array, bytes_transferred);
						for(list<ReceivedPacket>::iterator iter = received.begin(); iter != received.end(); ++iter)
						{
							Connection::PacketReceivedEvent evt(imp->client, *iter);
							imp->client->PacketReceived(&evt);
						}
						imp->inbox->ClearPackets();

						if(!imp->terminated)
							imp->AsyncReceive();
					}
					else
					{
						Connection::ConnectionErrorEvent evt(imp->client, error);
						imp->client->ConnectionError(&evt);
					}
				}

				if(ptr->CanDelete())
					delete ptr;
			}
		} my_receive_handler;

		struct MySendHandler
		{
			ImpPtr* ptr;
			vector<unsigned char> bytes;

			MySendHandler() : ptr(NULL), bytes(1024) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				unique_lock<mutex> lock(ptr->mutex);				// synchronize the following...

				ptr->send = false;
				Imp* imp = ptr->imp;

				if(imp != NULL)
				{
					if(!error)
					{
						vector<unsigned char> sent_bytes(bytes_transferred);
						for(unsigned int i = 0; i < bytes_transferred; ++i)
							sent_bytes[i] = bytes[i];

						Connection::BytesSentEvent evt(imp->client, sent_bytes);
						imp->client->BytesSent(&evt);
					}
					else
					{
						Connection::ConnectionErrorEvent evt(imp->client, error);
						imp->client->ConnectionError(&evt);
					}
				}

				if(ptr->CanDelete())
					delete ptr;
			}
		} my_send_handler;
	};

	void Client::Imp::AsyncReceive()
	{
		self->receive = true;
		socket->async_receive(buffer(my_receive_handler.buf), my_receive_handler);
	}




	/*
	 * Client methods
	 */
	Client::Client() : Connection()
	{
		imp = new Imp(this, &inbox);
		imp->my_connect_handler.ptr = imp->my_send_handler.ptr = imp->my_receive_handler.ptr = imp->self = new Imp::ImpPtr(imp); 
	}

	void Client::InnerDispose() { if(imp) { delete imp; imp = NULL; } }

	void Client::Connect(const string& server_ip_string, unsigned short port_num) { imp->Connect(server_ip_string, port_num); }
	void Client::Disconnect() { imp->Disconnect(); }
	bool Client::IsConnected() { return imp->IsConnected(); }

	void Client::Send(Packet p) { imp->Send(p); }

	unsigned int Client::GetClientID() { return imp->id; }
}
