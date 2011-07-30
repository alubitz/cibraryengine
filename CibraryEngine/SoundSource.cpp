#include "StdAfx.h"
#include "SoundSource.h"

#include "SoundSystem.h"
#include "SoundBuffer.h"

namespace CibraryEngine
{
	/*
	 * SoundSource methods
	 */
	SoundSource::SoundSource(SoundType sound_type, Vec3 pos, Vec3 vel, SoundBuffer* sound, float vol, bool looping, SoundSystem* sys) :
		sound_type(sound_type),
		loudness(0),
		valid(true),
		sys(sys),
		sound(sound),
		pos(pos),
		vel(vel),
		id(0)
	{
		SoundSystem::CheckForALErrors(__LINE__, __FILE__);

		alGenSources(1, &id);
		SoundSystem::CheckForALErrors(__LINE__, __FILE__);
		alSourcei(id, AL_BUFFER, sound->GetALName());
		SoundSystem::CheckForALErrors(__LINE__, __FILE__);
		alSourcei(id, AL_LOOPING, looping);

		SoundSystem::CheckForALErrors(__LINE__, __FILE__);

		SetPosVel();
		SetLoudness(vol);

		alSourcePlay(id);

		SoundSystem::CheckForALErrors(__LINE__, __FILE__);
	}

	void SoundSource::InnerDispose()
	{
		if (!sys->master_enable)
			return;

		alSourceStop(id);
		alDeleteSources(1, &id);
	}

	void SoundSource::SetPosVel()
	{
		if (!sys->master_enable)
			return;

		alSource3f(id, AL_POSITION, pos.x, pos.y, pos.z);
		alSource3f(id, AL_VELOCITY, vel.x, vel.y, vel.z); 
		SoundSystem::CheckForALErrors(__LINE__, __FILE__);
	}

	void SoundSource::InvalidateSound() { valid = false; }

	void SoundSource::Update(TimingInfo time)
	{
		if (!sys->master_enable)
			return;

		int state;
		alGetSourcei(id, AL_SOURCE_STATE, &state);

		SoundSystem::CheckForALErrors(__LINE__, __FILE__);

		if (state == AL_STOPPED)
		{
			InvalidateSound();
			return;
		}

		SetPosVel();

		pos += vel * time.elapsed;
	}

	void SoundSource::StopLooping()
	{
		if(!sys->master_enable)
			return;
		alSourcei(id, AL_LOOPING, false);
	}

	float SoundSource::GetLoudness() { return loudness; }
	void SoundSource::SetLoudness(float f)
	{
		if(!sys->master_enable)
			return;
		if (loudness != f)
		{
			loudness = f;

			alSourcef(id, AL_REFERENCE_DISTANCE, 10 * loudness);
			alSourcef(id, AL_MAX_DISTANCE, 100 * loudness);
			alSourcef(id, AL_GAIN, loudness);
			SoundSystem::CheckForALErrors(__LINE__, __FILE__);
		}
	}

	bool SoundSource::IsValid() { return valid; }
}
