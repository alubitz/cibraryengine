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
		Client* client;

		string server_ip_string;
		unsigned short port_num;

		unsigned int id;

		bool started;
		bool connected;
		bool terminated;

		Inbox* inbox;

		ip::tcp::socket* socket;

		Imp(Client* client, Inbox* inbox) :
			client(client),
			server_ip_string(),
			port_num(0),
			id(0),
			started(false),
			connected(false),
			terminated(false),
			inbox(inbox),
			socket(NULL),
			my_connect_handler(this),
			my_receive_handler(this),
			my_send_handler(this)
		{
		}

		~Imp() { Disconnect(); }

		void Connect(string server_ip_string, unsigned short port_num)
		{
			// TODO: synchronize this
			if(!started)
			{
				started = true;

				socket = new ip::tcp::socket(Network::GetIOService());

				ip::address address = ip::address_v4::from_string(server_ip_string.c_str());
				ip::tcp::endpoint endpoint(address, port_num);
				
				this->server_ip_string = server_ip_string;
				this->port_num = port_num;

				socket->async_connect(endpoint, my_connect_handler);
			}
		}

		void Disconnect()
		{
			// TODO: synchronize this
			if (connected && !terminated)
            {
                terminated = true;
                if (socket != NULL)
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
			// TODO: synchronize this
			if (connected && !terminated)
            {
				string bytes = p.GetBytes();
				for(unsigned int i = 0; i < bytes.length(); i++)
					my_send_handler.bytes[i] = bytes[i];

				socket->async_send(buffer(&my_send_handler.bytes[0], bytes.length() * sizeof(unsigned char)), my_send_handler);
            }
		}

		void AsyncReceive();

		struct MyConnectHandler
		{
			Imp* imp;
			MyConnectHandler(Imp* imp) : imp(imp) { }

			void operator ()(const boost::system::error_code& error) 
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
		} my_connect_handler;

		struct MyReceiveHandler
		{
			Imp* imp;
			vector<unsigned char> bytes;
			mutable_buffer buf;

			MyReceiveHandler(Imp* imp) : imp(imp), bytes(1024), buf(&bytes[0], 1024 * sizeof(unsigned char)) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				if(!error)
				{
					unsigned char* my_byte_array = buffer_cast<unsigned char*>(buf);

					vector<unsigned char> packet_bytes(bytes_transferred);
					for(unsigned int i = 0; i < bytes_transferred; i++)
						packet_bytes[i] = my_byte_array[i];

					Connection::BytesReceivedEvent evt(imp->client, packet_bytes);
					imp->client->BytesReceived(&evt);

					list<ReceivedPacket> received = imp->inbox->Receive(my_byte_array, bytes_transferred);
					for(list<ReceivedPacket>::iterator iter = received.begin(); iter != received.end(); iter++)
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
		} my_receive_handler;

		struct MySendHandler
		{
			Imp* imp;
			vector<unsigned char> bytes;
			MySendHandler(Imp* imp) : imp(imp), bytes(1024) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				if(!error)
				{
					vector<unsigned char> sent_bytes(bytes_transferred);
					for(unsigned int i = 0; i < bytes_transferred; i++)
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
		} my_send_handler;
	};

	void Client::Imp::AsyncReceive()
	{
		socket->async_receive(buffer(my_receive_handler.buf), my_receive_handler);
	}




	/*
	 * Client methods
	 */
	Client::Client() : Connection(), imp(new Imp(this, &inbox)) { }

	void Client::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void Client::Connect(string server_ip_string, unsigned short port_num) { imp->Connect(server_ip_string, port_num); }
	void Client::Disconnect() { imp->Disconnect(); }
	bool Client::IsConnected() { return imp->IsConnected(); }

	void Client::Send(Packet p) { imp->Send(p); }

	unsigned int Client::GetClientID() { return imp->id; }
}
