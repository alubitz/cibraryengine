#include "StdAfx.h"
#include "SoundBuffer.h"
#include "SoundSystem.h"

#include "DebugLog.h"

#include "Serialize.h"

namespace CibraryEngine
{
	/*
	 * SoundBuffer methods
	 */
	SoundBuffer::SoundBuffer(unsigned int al_name) : al_name(al_name) { }

	void SoundBuffer::InnerDispose()
	{
		if(al_name != 0)
		{
			alDeleteBuffers(1, &al_name);
			al_name = 0;
		}
	}

	bool SoundBuffer::IsLoaded() { return al_name != 0; }
	unsigned int SoundBuffer::GetALName() { return al_name; }

	SoundBuffer* SoundBuffer::FromFile(string filename)
	{
		unsigned int al_name = 0;

		ifstream file(filename.c_str(), ios::in | ios::binary);

		int channels, bits_per_sample, sample_rate;

		vector<char> sound_data;
		unsigned int error = LoadWave(file, channels, bits_per_sample, sample_rate, &sound_data);
		if(error == 0)
		{
			ALenum format = GetSoundFormat(channels, bits_per_sample);
			if(format != 0)
			{
				alGenBuffers(1, &al_name);

				unsigned int size = sound_data.size();
				alBufferData(al_name, format, &sound_data[0], size, sample_rate);
			}
			else
			{
				stringstream message;
				message << "format unsupported: channels=" << channels << ", bits per sample=" << bits_per_sample << ", in file \"" << filename << "\"" << endl;
				Debug(message.str());

				return NULL;
			}
		}
		else
		{
			stringstream message;
			message << "error " << error << " loading sound from file: \"" << filename << "\"" << endl;
			Debug(message.str());

			return NULL;
		}

		if(SoundSystem::CheckForALErrors())
		{
			alDeleteBuffers(1, &al_name);
			al_name = 0;
		}

		if(al_name == 0)
		{
			stringstream message;
			message << "failure to load file: \"" << filename << "\"" << endl;
			Debug(message.str());

			return NULL;
		}
		else
			return new SoundBuffer(al_name);
	}

	unsigned int SoundBuffer::LoadWave(istream& stream, int& channels, int& bits, int& rate, vector<char> *buffer)
	{
		*buffer = vector<char>();

		if(!stream)
			return 1;

		//get filesize
		streamsize size = 0;
		if(stream.seekg(0, ios::end).good())
			size = stream.tellg();
		if(stream.seekg(0, ios::beg).good())
			size -= stream.tellg();

		if(size <= 0)
			return 2;

		stream.seekg(0, ios::beg);			// reset to the beginning of the stream

		// RIFF header
		string signature = "";
		for(unsigned int i = 0; i < 4; i++)
			signature += ReadByte(stream);
		if (signature != "RIFF")
			return 3;

		int riff_chunk_size = FlipInt32(ReadInt32(stream));

		string format = "";
		for(unsigned int i = 0; i < 4; i++)
			format += ReadByte(stream);
		if (format != "WAVE")
			return 4;

		// WAVE header
		string format_signature = "";
		for(unsigned int i = 0; i < 4; i++)
			format_signature += ReadByte(stream);
		if (format_signature != "fmt ")
			return 5;

		int format_chunk_size = FlipInt32(ReadInt32(stream));
		short int audio_format = FlipInt16(ReadInt16(stream));
		short int num_channels = FlipInt16(ReadInt16(stream));
		int sample_rate = FlipInt32(ReadInt32(stream));
		int byte_rate = FlipInt32(ReadInt32(stream));
		short int block_align = FlipInt16(ReadInt16(stream));
		short int bits_per_sample = FlipInt16(ReadInt16(stream));

		string data_signature = "";
		for(unsigned int i = 0; i < 4; i++)
			data_signature += ReadByte(stream);
		if (data_signature != "data")
			return 6;
		int data_chunk_size = FlipInt32(ReadInt32(stream));

		channels = num_channels;
		bits = bits_per_sample;
		rate = sample_rate;

		vector<char> temp_buffer = vector<char>();

		unsigned int buffer_size = size - stream.tellg();
		temp_buffer.resize(buffer_size);
		stream.read(&temp_buffer[0], buffer_size);

		*buffer = temp_buffer;
		return 0;
	}

	ALenum SoundBuffer::GetSoundFormat(int channels, int bits)
	{
		switch (channels)
		{
			case 1: return bits == 8 ? AL_FORMAT_MONO8 : AL_FORMAT_MONO16;
			case 2: return bits == 8 ? AL_FORMAT_STEREO8 : AL_FORMAT_STEREO16;
			default: return 0;
		}
	}



	/*
	 * SoundBufferLoader methods
	 */
	SoundBuffer* SoundBufferLoader::Load(ContentMetadata& what)
	{
		return SoundBuffer::FromFile("Files/Sound/" + what.name + ".wav");
	}

	void SoundBufferLoader::Unload(SoundBuffer* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}
}
