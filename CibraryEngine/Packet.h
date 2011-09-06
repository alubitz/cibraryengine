#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct Packet
	{
	private:

		vector<unsigned char> data;

	public:

		// member methods
		Packet();
		Packet(vector<unsigned char> data);

		unsigned int GetContentLength();
		vector<unsigned char> GetContentBytes();

		bool DecodePacket(string& type, vector<unsigned char>& data);

		// static methods
		static Packet CreateAutoLength(vector<unsigned char>& data);
		static Packet CreateFixedLength(string type, unsigned char* data, unsigned int len);

		static bool MaybeExtractPacket(vector<unsigned char>& byte_stream, vector<unsigned char>& unused_bytes, Packet& packet_out);
	};
}
