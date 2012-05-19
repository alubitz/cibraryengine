#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	/**
	 * Generic binary-format chunks
	 */
	class BinaryChunk
	{
		private:

			string name;

		public:

			string data;

			BinaryChunk();
			BinaryChunk(const string& name);

			void Read(istream& stream);
			void Write(ostream& stream);

			string GetName();
			void SetName(const string& n);
	};

	struct ChunkTypeFunction
	{
		virtual void HandleChunk(BinaryChunk& chunk) { }
	};

	struct ChunkTypeIndexer : public ChunkTypeFunction
	{
		map<string, ChunkTypeFunction*> chunk_handlers;
		ChunkTypeFunction* default_handler;

		ChunkTypeIndexer();

		void HandleChunk(BinaryChunk& chunk);

		void SetHandler(const string& name, ChunkTypeFunction* func);
		void SetDefaultHandler(ChunkTypeFunction* func);
	};
}
