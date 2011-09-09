#include "StdAfx.h"

#include <boost/asio.hpp>

#include "Client.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	using namespace boost::asio;



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

		Inbox* inbox;

		ip::tcp::socket* socket;

		Imp(string server_ip_string, int port_num, Inbox* inbox) :
			server_ip_string(server_ip_string),
			port_num(port_num),
			id(0),
			started(false),
			connected(false),
			terminated(false),
			inbox(inbox),
			socket(NULL)
		{
		}

		~Imp() { Disconnect(); }

		void Connect()
		{
			// TODO: synchronize this
			if(!started)
			{
				started = true;

				socket = new ip::tcp::socket(io_service());

				ip::address address = ip::address_v4::from_string(server_ip_string.c_str());
				ip::tcp::endpoint endpoint(address, port_num);
				
				// TODO: socket->async_connect(endpoint, ???);
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
                    // TODO: trigger client disconected event

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
				vector<unsigned char> bytes = p.GetBytes();

				socket->async_send(buffer(&bytes[0], bytes.size() * sizeof(unsigned char)), MySendHandler(this));
            }
		}

		struct MyConnectHandler
		{
			Imp* imp;
			MyConnectHandler(Imp* imp) : imp(imp) { }

			void operator ()(const boost::system::error_code& error) 
			{
				if(!error)
				{
					imp->connected = true;

					// TODO: trigger client connected event here

					MyReceiveHandler handler(imp);
					imp->socket->async_receive(buffer(handler.buf), handler);
				}
			}
		};

		struct MyReceiveHandler
		{
			Imp* imp;
			mutable_buffer buf;

			MyReceiveHandler(Imp* imp) : imp(imp), buf() { }


			void operator ()(const boost::system::error_code& error, size_t bytes_transferred)
			{
				if(!error)
				{
					unsigned char* my_byte_array = buffer_cast<unsigned char*>(buf);
					
					// TODO: trigger bytes received event

					list<ReceivedPacket> received = imp->inbox->Receive(my_byte_array, bytes_transferred);
					for(list<ReceivedPacket>::iterator iter = received.begin(); iter != received.end(); iter++)
					{
						// TODO: trigger packet received event
					}
					imp->inbox->ClearPackets();

					if(!imp->terminated)
					{
						MyReceiveHandler handler(imp);
						imp->socket->async_receive(buffer(handler.buf), handler);
					}
				}
			}
		};

		struct MySendHandler
		{
			Imp* imp;
			MySendHandler(Imp* imp) : imp(imp) { }

			void operator ()(const boost::system::error_code& error, size_t bytes_transferred) { /* TODO: trigger a packet sent event */ }
		};
	};




	/*
	 * Client methods
	 */
	Client::Client(string server_ip_string, int port_num) : Connection(), imp(new Imp(server_ip_string, port_num, &inbox)) { }

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
