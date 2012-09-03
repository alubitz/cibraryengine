#include "StdAfx.h"

#include "Packet.h"
#include "Serialize.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * Packet methods
	 */
	Packet::Packet() : data() { }
	Packet::Packet(string data) : data(data) { }

	string Packet::GetBytes() { return data; }

	unsigned int Packet::GetContentLength()
	{
		istringstream ss(data);
		return ReadUInt32(ss);
	}

	string Packet::GetContentBytes()
	{
		string result;
		unsigned int len = max(data.length() - 4, 0u);
		for(unsigned int i = 0; i < len; ++i)
			result += data[i + 4];

		return result;
	}

	bool Packet::DecodePacket(string& type, string& out_data)
	{
		if(data.length() >= 12)
        {
            type = string();
            for(int i = 0; i < 8; ++i)
                type += data[i + 4];

			unsigned int len = data.size() - 12;
            out_data = string();
			for(unsigned int i = 0; i < len; ++i)
				out_data += data[i + 12];

            return true;
        }
        else
            return false;
	}

	Packet Packet::CreateAutoLength(string data)
	{
		stringstream ss;

		WriteUInt32(data.size(), ss);
		for(unsigned int i = 0; i < data.length(); ++i)
			WriteByte(data[i], ss);

        return Packet(ss.str());
	}

	Packet Packet::CreateNamedAutoLength(string type, string data)
	{
		if(type.length() > 8)
			type = type.substr(0, 8);
		else
		{
			while(type.length() < 8)
				type += '_';
		}

		unsigned int len = data.length();

		stringstream ss;
		
		WriteUInt32(len + 8, ss);
		for(unsigned int i = 0; i < 8; ++i)
			WriteByte(type[i], ss);
		for(unsigned int i = 0; i < len; ++i)
			WriteByte(data[i], ss);

        return Packet(ss.str());
	}

	bool Packet::MaybeExtractPacket(string& byte_stream, string& unused_bytes, Packet& packet_out)
	{
		if(byte_stream.length() < 4)
        {
            unused_bytes = byte_stream;
            return false;
        }
		
		istringstream ss(byte_stream);
        unsigned int len = ReadUInt32(ss);

        if(byte_stream.length() < len + 4)
        {
            unused_bytes = byte_stream;
            return false;
        }
        else
        {
			string packet_bytes = byte_stream.substr(0, len + 4);
			packet_out = Packet(packet_bytes);

			unused_bytes = byte_stream.substr(len + 4);

            return true;
        }
	}
}
