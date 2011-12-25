#include "StdAfx.h"
#include "Serialize.h"

namespace CibraryEngine
{
	bool little_endian = true;			// defaults to true; make sure to call InitEndianness, or it might mess up
	void InitEndianness()
	{
		short int word = 0x0001;
		char *byte = (char*)&word;

		little_endian = byte[0] != 0;
	}

	int GetFileString(string filename, string* result)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		//get filesize
		streamsize size = 0;
		if(file.seekg(0, ios::end).good()) size = file.tellg();
		if(file.seekg(0, ios::beg).good()) size -= file.tellg();

		vector<char> buffer = vector<char>();
		//read contents of the file into the vector
		buffer.resize(size_t(size));
		if(size > 0) file.read((char*)(&buffer[0]), size);

		file.close();

		result->assign(&buffer[0], (size_t)size);
		return 0;
	}

	void WriteByte(unsigned char b, ostream& stream) { stream.put(b); }
	unsigned char ReadByte(istream& stream) { return stream.get(); }

	void WriteBool(bool b, ostream& stream) { stream.put(b ? 1 : 0); }
	bool WriteBool(istream& stream) { return stream.get() != 0; }

	void WriteUInt16(unsigned short int i, ostream& stream)
	{
		WriteByte((i & (0xFF << 8)) >> 8, stream);
		WriteByte((i & (0xFF << 0)) >> 0, stream);
	}
	unsigned short int ReadUInt16(istream& stream)
	{
		unsigned char a = ReadByte(stream), b = ReadByte(stream);
		unsigned short int result = (a << 8) | (b << 0);
		return result;
		//return (ReadByte(stream) << 8) | (ReadByte(stream) << 0);
	}

	void WriteInt16(short int i, ostream& stream) { WriteUInt16((unsigned short int)i, stream); }
	short int ReadInt16(istream& stream) { return (short int)ReadUInt16(stream); }

	void WriteUInt32(unsigned int i, ostream& stream)
	{
		WriteByte((i & (0xFF << 24)) >> 24, stream);
		WriteByte((i & (0xFF << 16)) >> 16, stream);
		WriteByte((i & (0xFF << 8)) >> 8, stream);
		WriteByte((i & (0xFF << 0)) >> 0, stream);
	}
	unsigned int ReadUInt32(istream& stream)
	{
		return (ReadByte(stream) << 24) | (ReadByte(stream) << 16) | (ReadByte(stream) << 8) | (ReadByte(stream) << 0);
	}

	void WriteInt32(int i, ostream& stream) { WriteUInt32((unsigned int)i, stream); }
	int ReadInt32(istream& stream) { return (int)ReadUInt32(stream); }

	void WriteSingle(float f, ostream& stream) { WriteUInt32(*(unsigned int *)&f, stream); }
	float ReadSingle(istream& stream) { unsigned int i = ReadUInt32(stream); return *((float *)&i); }

	void WriteString1(string s, ostream& stream)
	{
		unsigned char size = s.length();
		WriteByte(size, stream);
		for(unsigned int i = 0; i < size; ++i)
			WriteByte(s[i], stream);
	}
	void WriteString4(string s, ostream& stream)
	{
		unsigned int size = s.length();
		WriteUInt32(size, stream);
		for(unsigned int i = 0; i < size; ++i)
			WriteByte(s[i], stream);
	}
	string ReadString1(istream& stream)
	{
		unsigned char size = ReadByte(stream);
		string s;
		for(unsigned int i = 0; i < size; ++i)
			s += ReadByte(stream);
		return s;
	}
	string ReadString4(istream& stream)
	{
		unsigned int size = ReadUInt32(stream);
		string s;
		for(unsigned int i = 0; i < size; ++i)
			s += ReadByte(stream);
		return s;
	}

	unsigned short int FlipUInt16(unsigned short int input)
	{
		return (((input >> 8) & 0xFF) << 0) | (((input >> 0) & 0xFF) << 8);
	}

	unsigned int FlipUInt32(unsigned int input)
	{
		return (((input >> 24) & 0xFF) << 0) | (((input >> 16) & 0xFF) << 8) | (((input >> 8) & 0xFF) << 16) | (((input >> 0) & 0xFF) << 24);
	}

	short int FlipInt16(short int input) { return (short int)FlipUInt16((unsigned short int)input); }
	int FlipInt32(int input) { return (int)FlipUInt32((unsigned int)input); }
}
