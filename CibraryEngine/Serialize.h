// this is equivalent to what http://www.parashift.com/c++-faq-lite/serialization.html#faq-36.6 would have me call "machine.h"

#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	void InitEndianness();			// This seems to have been rendered unnecessary by that clever bitshifting solution I came up with

	/** Makes a string containing the entire contents of a file; returns 0 if ok, or a nonzero int error code */
	int GetFileString(string filename, string* result);

	/** Writes a boolean to a stream */
	void WriteBool(bool b, ostream& stream);
	/** Reads a boolean from a stream */
	bool ReadBool(istream& stream);

	/** Writes a byte to a stream */
	void WriteByte(unsigned char b, ostream& stream);
	/** Reads a byte from a stream */
	unsigned char ReadByte(istream& stream);

	/** Writes an unsigned short int to a stream */
	void WriteUInt16(unsigned short int i, ostream& stream);
	/** Reads an unsigned short int from a stream */
	unsigned short int ReadUInt16(istream& stream);

	/** Writes a signed short int to a stream */
	void WriteInt16(short int i, ostream& stream);
	/** Reads a signed short int from a stream */
	short int ReadInt16(istream& stream);

	/** Writes an unsigned int to a stream */
	void WriteUInt32(unsigned int i, ostream& stream);
	/** Reads an unsigned int from a stream */
	unsigned int ReadUInt32(istream& stream);

	/** Writes a signed int to a stream */
	void WriteInt32(int i, ostream& stream);
	/** Reads a signed int from a stream */
	int ReadInt32(istream& stream);

	/** Writes a single-precision floating point number to a stream */
	void WriteSingle(float f, ostream& stream);
	/** Reads a single-precision floating point number from a stream */
	float ReadSingle(istream& stream);

	void WriteString1(string s, ostream& stream);
	void WriteString4(string s, ostream& stream);
	string ReadString1(istream& stream);
	string ReadString4(istream& stream);

	/** Changes the endianness of an unsigned short int */
	unsigned short int FlipUInt16(unsigned short int input);
	/** Changes the endianness of an unsigned int */
	unsigned int FlipUInt32(unsigned int input);
	/** Changes the endianness of a signed short int */
	short int FlipInt16(short int input);
	/** Changes the endianness of a signed int */
	int FlipInt32(int input);
}
