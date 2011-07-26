#include "BinaryChunk.h"

#include "Serialize.h"

namespace CibraryEngine
{
	/*
	 * BinaryChunk methods
	 */
	BinaryChunk::BinaryChunk() : name(), data() { }
	BinaryChunk::BinaryChunk(string name) : data() { SetName(name); }

	void BinaryChunk::Read(istream& stream)
	{
		name = "";
		for(int i = 0; i < 8; i++)
			name += ReadByte(stream);

		unsigned int size = ReadUInt32(stream);
		data = "";
		data.resize(size);
		stream.read(&data[0], size);
	}

	void BinaryChunk::Write(ostream& stream)
	{
		for(int i = 0; i < 8; i++)
			WriteByte(name[i], stream);

		WriteUInt32(data.length(), stream);
		stream.write(&data[0], data.length());
	}

	string BinaryChunk::GetName() { return name; }

	void BinaryChunk::SetName(string n) { name = n; }




	/*
	 * ChunkTypeIndexer methods
	 */
	ChunkTypeIndexer::ChunkTypeIndexer() : ChunkTypeFunction(), chunk_handlers(), default_handler(NULL) { }

	void ChunkTypeIndexer::HandleChunk(BinaryChunk& chunk)
	{
		istringstream ss(chunk.data);
		while(ss.tellg() < chunk.data.length())
		{
			BinaryChunk part;
			part.Read(ss);

			map<string, ChunkTypeFunction*>::iterator found = chunk_handlers.find(part.GetName());
			if(found != chunk_handlers.end())
			{
				ChunkTypeFunction* func = found->second;
				func->HandleChunk(part);
			}
			else if(default_handler != NULL)
				default_handler->HandleChunk(part);
		}
	}

	void ChunkTypeIndexer::SetHandler(string name, ChunkTypeFunction* func)
	{
		chunk_handlers[name] = func;
	}

	void ChunkTypeIndexer::SetDefaultHandler(ChunkTypeFunction* func) { default_handler = func; }
}
