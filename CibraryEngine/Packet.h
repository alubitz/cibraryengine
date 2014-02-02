#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct Packet
	{
	private:

		string data;

	public:

		// member methods
		Packet();
		Packet(const string& data);

		string GetBytes();

		unsigned int GetContentLength();
		string GetContentBytes();

		bool DecodePacket(string& type, string& data);

		// static methods
		static Packet CreateAutoLength(const string& data);
		static Packet CreateNamedAutoLength(const string& type, const string& data);

		static bool MaybeExtractPacket(const string& byte_stream, string& unused_bytes, Packet& packet_out);
	};
}
