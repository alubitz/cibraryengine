#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "Content.h"

namespace CibraryEngine
{
	using namespace std;

	class SoundBuffer : public Disposable
	{
		private:

			unsigned int al_name;

		protected:

			void InnerDispose();

			SoundBuffer(unsigned int al_name);

		public:

			unsigned int GetALName();

			// not overriding Load or RecommendUnload; all of our loading is done via SoundBuffer::FromFile
			bool IsLoaded();

			static SoundBuffer* FromFile(const string& filename);
			static unsigned int LoadWave(istream& stream, int& channels, int& bits, int& rate, vector<char>* buffer);
			static ALenum GetSoundFormat(int channels, int bits);
	};

	struct SoundBufferLoader : public ContentTypeHandler<SoundBuffer>
	{
		SoundBufferLoader(ContentMan* man) : ContentTypeHandler<SoundBuffer>(man) { }

		SoundBuffer* Load(ContentMetadata& what);
		void Unload(SoundBuffer* content, ContentMetadata& meta);
	};
}
