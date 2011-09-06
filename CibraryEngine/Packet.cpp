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
	Packet::Packet(vector<unsigned char> data) : data(data) { }

	unsigned int Packet::GetContentLength()
	{
		string s;
		for(unsigned int i = 0; i < data.size(); i++)
			s += data[i];

		istringstream ss(s);

		return ReadUInt32(ss);
	}

	vector<unsigned char> Packet::GetContentBytes()
	{
		vector<unsigned char> result(max(4, data.size()) - 4);
		for(unsigned int i = 0; i < result.size(); i++)
			result[i] = data[i + 4];

		return result;
	}

	bool Packet::DecodePacket(string& type, vector<unsigned char>& out_data)
	{
		if (data.size() >= 12)
        {
            type = "";
            for (int i = 0; i < 8; i++)
                type += (char)data[i + 4];

            out_data = vector<unsigned char>(data.size() - 12);
			for(unsigned int i = 0; i < out_data.size(); i++)
				out_data[i] = data[i + 12];

            return true;
        }
        else
            return false;
	}

	Packet Packet::CreateAutoLength(vector<unsigned char>& data)
	{
		stringstream ss;

		WriteUInt32(data.size(), ss);
		for(unsigned int i = 0; i < data.size(); i++)
			WriteByte(data[i], ss);
		
		string s = ss.str();

		vector<unsigned char> bytes;
		for(unsigned int i = 0; i < s.length(); i++)
			bytes.push_back(s[i]);

        return Packet(bytes);
	}

	Packet Packet::CreateFixedLength(string type, unsigned char* data, unsigned int len)
	{
		if(type.length() > 8)
			type = type.substr(0, 8);
		else
		{
			while(type.length() < 8)
				type += '_';
		}

		stringstream ss;
		
		WriteUInt32(len + 8, ss);
		for(unsigned int i = 0; i < 8; i++)
			WriteByte(type[i], ss);
		for(unsigned int i = 0; i < len; i++)
			WriteByte(data[i], ss);

		string s = ss.str();

		vector<unsigned char> bytes(s.length());

		for(unsigned int i = 0; i < s.length(); i++)
			bytes[i] = s[i];

        return Packet(bytes);
	}

	bool Packet::MaybeExtractPacket(vector<unsigned char>& byte_stream, vector<unsigned char>& unused_bytes, Packet& packet_out)
	{
		if (byte_stream.size() < 4)
        {
            unused_bytes = byte_stream;
            return false;
        }
		
		string s;
		for(int i = 0; i < 4; i++)
			s += byte_stream[i];
		istringstream ss(s);

        unsigned int len = ReadUInt32(ss);

        if (byte_stream.size() < len + 4)
        {
            unused_bytes = byte_stream;
            return false;
        }
        else
        {
			vector<unsigned char> packet_bytes(len + 4);
			for(unsigned int i = 0; i < len + 4; i++)
				packet_bytes[i] = byte_stream[i];

			packet_out = Packet(packet_bytes);

			// doing this just in case the two reference params are the same
			vector<unsigned char> extra_bytes(byte_stream.size() - (len + 4));
			for(unsigned int i = 0; i < byte_stream.size() - (len + 4); i++)
				extra_bytes[i] = byte_stream[i + len + 4];
            unused_bytes = extra_bytes;

            return true;
        }
	}
}
