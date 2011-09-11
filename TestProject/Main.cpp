#include "StdAfx.h"

#include "MainMenu.h"
#include "LoadingScreen.h"

using namespace std;

using namespace CibraryEngine;
using namespace Test;

int main(int argc, char** argv)
{
	ScriptSystem::Init();

	InitEndianness();

	Network::StartAsyncSystem();

	Server server;

	struct IncomingConnectionHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Debug("Server event: incoming connection\n");
		}
	} incoming_connection;

	struct BeganListeningHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Debug("Server event: began listening\n");
		}
	} began_listening;

	struct ServerErrorHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			stringstream ss;
			ss << "server error: " << ((Server::ServerErrorEvent*)evt)->error << endl;
			Debug(ss.str());
		}
	} server_error_handler;

	struct ConnectionErrorHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			stringstream ss;
			ss << "connection error: " << ((Connection::ConnectionErrorEvent*)evt)->error << endl;
			Debug(ss.str());
		}
	} connection_error_handler;

	struct ServerReceivedPacketHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Server::PacketReceivedEvent* pre = (Server::PacketReceivedEvent*)evt;
			ReceivedPacket packet = pre->packet;

			string type;
			string data;
			packet.packet.DecodePacket(type, data);

			stringstream ss;
			ss << "server received packet; type is \"" << type << "\"; packet body contains " << data.length() << " bytes of data" << endl;
			Debug(ss.str());
		}
	} server_received_packet_handler;

	struct ServerBytesReceivedHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Server::BytesReceivedEvent* bre = (Server::BytesReceivedEvent*)evt;

			stringstream ss;
			ss << "server received " << bre->bytes.size() << " bytes from client" << endl;
			Debug(ss.str());
		}
	} server_bytes_received_handler;

	struct ClientErrorHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			stringstream ss;
			ss << "client error: " << ((Connection::ConnectionErrorEvent*)evt)->error << endl;
			Debug(ss.str());
		}
	} client_error_handler;

	struct ClientBytesSentHandler : public EventHandler
	{
		void HandleEvent(Event* evt)
		{
			Connection::BytesSentEvent* bse = (Connection::BytesSentEvent*)evt;

			stringstream ss;
			ss << "client sent " << bse->bytes.size() << " bytes" << endl;
			Debug(ss.str());
		}
	} client_bytes_sent_handler;

	server.IncomingConnection += &incoming_connection;
	server.BeganListening += &began_listening;
	server.ServerError += &server_error_handler;
	server.ConnectionError += &connection_error_handler;
	server.PacketReceived += &server_received_packet_handler;
	server.BytesReceived += &server_bytes_received_handler;

	server.Start(7777);

	while(!server.IsActive()) { }

	Client client;
	client.ConnectionError += &client_error_handler;
	client.BytesSent += &client_bytes_sent_handler;

	client.Connect("127.0.0.1", 7777);

	while(!client.IsConnected()) { }

	client.Send(Packet::CreateNamedAutoLength("HELLO", string()));

	ProgramWindow* win = ProgramWindow::CreateProgramWindow("C++ Game Engine", 0, 0, 0, false);
	if(win == NULL)
		return 1;

	ScreenshotGrabber grabber(win);
	win->input_state->KeyStateChanged += &grabber;

	ProgramScreen* first_screen = new MainMenu(win, NULL);

	int result = win->Run(first_screen);

	win->input_state->KeyStateChanged -= &grabber;

	delete first_screen;
	delete win;

	client.Disconnect();
	server.Disconnect();

	return result;
}

#ifdef WIN32
int WINAPI WinMain(	HINSTANCE	hInstance,		// Instance
					HINSTANCE	hPrevInstance,	// Previous Instance
					LPSTR		lpCmdLine,		// Command Line Parameters
					int			nCmdShow)		// Window Show State
{
	return main(nCmdShow, &lpCmdLine);
}
#endif
